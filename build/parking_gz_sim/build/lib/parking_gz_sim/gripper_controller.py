#!/usr/bin/env python3
"""
gripper_controller.py — 초음파 바퀴 감지 + 그리퍼 파지 + 차량 결합(리프트 대역)

흐름:
  INIT_DETACH : DetachableJoint가 attach 상태로 스폰되므로 기동 직후 분리
  WAIT        : /sync/goal_reached 수신 + 그립 위치 근접 → 시퀀스 시작
                (이때 /sync/enable=False로 주행 제어권을 가져옴)
  SCAN_BACK   : 쌍을 -x로 후퇴 — 초음파가 바퀴 에코를 벗어난 지점에서 시작
  SCAN_FWD    : +x 저속 전진하며 초음파 "값이 바뀌는 지점"(에지) 기록
                에코 시작/끝의 중점 = 바퀴 중심
  CENTER      : 바퀴 중심으로 정렬 (front 로봇 중심 = 앞 축)
  GRIP        : 그리퍼 4개 180°(접힘) → 90°(전개) — 바퀴 앞뒤 케이지
  ATTACH      : DetachableJoint attach → 차량 결합 (리프트 완료 대역)
  DONE        : /robot/lifted=True 발행 (latch), /sync/enable=True 복구
                이후 운반 주행은 기존 sim_rigid_body_sync가 수행

초음파 사용 규칙 (아두이노 HC-SR04 대역):
  LaserScan 5-ray(빔폭 15°)의 min 하나만 단일 거리값으로 사용한다.
  각도 정보는 쓰지 않는다 — 실제 초음파와 동일한 정보량 유지.

실전 대응:
  초음파 에지 검출   ↔ 아두이노 초음파로 바퀴 위치 검출
  DetachableJoint    ↔ 그리퍼 리프트 (바퀴 클램핑)
  /robot/lifted      ↔ 실전 명세의 리프트 완료 → 주행 허가 게이트
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Empty, Float64


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class GripperController(Node):
    def __init__(self):
        super().__init__('gripper_controller')

        # 초음파 에코 판정 임계 [m] — 정렬 시 바퀴 안쪽면까지 5.5cm
        self.declare_parameter('echo_threshold', 0.10)
        # 스캔 시작 전 후퇴량 [m] — 바퀴 사이 무에코 구간(0.55~0.70)으로
        self.declare_parameter('scan_back_dist', 0.13)
        # 스캔 최대 전진량 [m] (바퀴 못 찾으면 중단)
        self.declare_parameter('scan_max_dist', 0.35)
        self.declare_parameter('creep_speed', 0.012)
        self.declare_parameter('center_tol', 0.005)
        # 그리퍼 전개 각도 [rad] = 90°
        self.declare_parameter('grip_angle', 1.5708)
        self.declare_parameter('grip_settle_time', 3.0)
        # 그립 시퀀스 시작 인정 반경 [m] (target_pose 기준)
        self.declare_parameter('start_tol', 0.25)

        gp = self.get_parameter
        self.echo_th = float(gp('echo_threshold').value)
        self.back_dist = float(gp('scan_back_dist').value)
        self.scan_max = float(gp('scan_max_dist').value)
        self.creep_v = float(gp('creep_speed').value)
        self.center_tol = float(gp('center_tol').value)
        self.grip_angle = float(gp('grip_angle').value)
        self.grip_settle = float(gp('grip_settle_time').value)
        self.start_tol = float(gp('start_tol').value)

        # ===== 상태 =====
        self.state = 'INIT_DETACH'
        self.state_t0 = None
        self.front = None       # (x, y, yaw) world
        self.rear = None
        self.target_xy = None
        self.goal_reached = False
        self.lifted = False
        self.scan_origin = None
        # 에지 기록: {'L': {...}, 'R': {...}}
        self.edge = None
        self.wheel_x = None
        self.us = {'L': 99.0, 'R': 99.0}   # front robot 좌/우 최신 거리값

        # ===== 통신 =====
        self.create_subscription(Odometry, '/front/odom',
                                 lambda m: self.odom_cb('front', m), 20)
        self.create_subscription(Odometry, '/rear/odom',
                                 lambda m: self.odom_cb('rear', m), 20)
        self.create_subscription(Bool, '/sync/goal_reached',
                                 self.reached_cb, 10)
        self.create_subscription(LaserScan, '/front/us_left',
                                 lambda m: self.us_cb('L', m), 10)
        self.create_subscription(LaserScan, '/front/us_right',
                                 lambda m: self.us_cb('R', m), 10)
        self.create_subscription(PoseStamped, '/parking/target_pose',
                                 self.target_cb, 10)

        self.pub_f = self.create_publisher(Twist, '/front/cmd_vel', 10)
        self.pub_r = self.create_publisher(Twist, '/rear/cmd_vel', 10)
        self.pub_enable = self.create_publisher(Bool, '/sync/enable', 10)
        self.pub_lifted = self.create_publisher(Bool, '/robot/lifted', 10)
        self.pub_grips = [
            self.create_publisher(Float64, t, 10) for t in (
                '/front/grip_left_cmd', '/front/grip_right_cmd',
                '/rear/grip_left_cmd', '/rear/grip_right_cmd')
        ]
        self.pub_detach = [
            self.create_publisher(Empty, t, 10) for t in (
                '/front/vehicle_detach', '/rear/vehicle_detach')
        ]
        self.pub_attach = [
            self.create_publisher(Empty, t, 10) for t in (
                '/front/vehicle_attach', '/rear/vehicle_attach')
        ]

        self.create_timer(0.03, self.tick)
        self.create_timer(0.5, self.publish_status)
        self.get_logger().info('gripper_controller 시작 — 초기 분리 후 대기')

    # ================= 콜백 =================
    def odom_cb(self, role, msg):
        p = msg.pose.pose.position
        yaw = yaw_from_quat(msg.pose.pose.orientation)
        if role == 'front':
            self.front = (float(p.x), float(p.y), yaw)
        else:
            self.rear = (float(p.x), float(p.y), yaw)

    def us_cb(self, side, msg):
        """HC-SR04 대역: 스캔 전체의 min 하나만 단일 거리값으로 사용."""
        valid = [r for r in msg.ranges
                 if msg.range_min <= r <= msg.range_max]
        self.us[side] = min(valid) if valid else 99.0

    def reached_cb(self, msg):
        if msg.data:
            self.goal_reached = True

    def target_cb(self, msg):
        self.target_xy = (msg.pose.position.x, msg.pose.position.y)

    # ================= 유틸 =================
    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def elapsed(self):
        return 0.0 if self.state_t0 is None else self.now_s() - self.state_t0

    def goto(self, state):
        self.get_logger().info(f'[{self.state}] → [{state}]')
        self.state = state
        self.state_t0 = self.now_s()

    def creep(self, vx_world):
        """두 로봇에 동일한 world x 속도 — 각 로봇 body frame으로 변환."""
        for pose, pub in ((self.front, self.pub_f), (self.rear, self.pub_r)):
            t = Twist()
            if pose is not None:
                yaw = pose[2]
                t.linear.x = math.cos(yaw) * vx_world
                t.linear.y = -math.sin(yaw) * vx_world
            pub.publish(t)

    def stop(self):
        self.creep(0.0)

    def hold_sync(self, enabled):
        self.pub_enable.publish(Bool(data=enabled))

    # ================= 메인 =================
    def tick(self):
        if self.state_t0 is None:
            self.state_t0 = self.now_s()

        handler = getattr(self, 'st_' + self.state.lower())
        handler()

    def st_init_detach(self):
        # 플러그인이 attach 상태로 시작 → 1초간 detach 반복 발행
        for p in self.pub_detach:
            p.publish(Empty())
        if self.elapsed() > 1.0:
            self.get_logger().info('초기 분리 완료 — 그립 위치 도착 대기')
            self.goto('WAIT')

    def st_wait(self):
        if not self.goal_reached or self.lifted:
            return
        if self.front is None or self.rear is None or self.target_xy is None:
            return
        cx = (self.front[0] + self.rear[0]) / 2
        cy = (self.front[1] + self.rear[1]) / 2
        d = math.hypot(cx - self.target_xy[0], cy - self.target_xy[1])
        if d > self.start_tol:
            # 최종 목표 도착 등 다른 goal_reached — 무시
            self.goal_reached = False
            return
        self.get_logger().info(
            f'그립 위치 도착 확인 (오차 {d * 100:.1f}cm) — 초음파 정렬 시작')
        self.scan_origin = self.front[0]
        self.goto('SCAN_BACK')

    def st_scan_back(self):
        self.hold_sync(False)
        if self.front[0] > self.scan_origin - self.back_dist:
            self.creep(-2.0 * self.creep_v)
            return
        self.stop()
        self.scan_origin = self.front[0]
        self.edge = {s: {'in': False, 'enter': None, 'center': None}
                     for s in ('L', 'R')}
        self.goto('SCAN_FWD')

    def st_scan_fwd(self):
        self.hold_sync(False)
        x = self.front[0]

        for s in ('L', 'R'):
            e = self.edge[s]
            echo = self.us[s] < self.echo_th
            if echo and not e['in']:
                e['in'] = True
                e['enter'] = x
                self.get_logger().info(
                    f'초음파[{s}] 에코 시작 x={x:.3f} (거리 {self.us[s]:.3f}m)')
            elif not echo and e['in'] and e['center'] is None:
                e['center'] = (e['enter'] + x) / 2
                self.get_logger().info(
                    f'초음파[{s}] 에코 종료 x={x:.3f} → 바퀴 중심 {e["center"]:.3f}')

        centers = [e['center'] for e in self.edge.values()
                   if e['center'] is not None]
        if len(centers) == 2:
            self.stop()
            self.wheel_x = sum(centers) / 2
            self.get_logger().info(f'앞바퀴 축 중심 확정: x={self.wheel_x:.3f}')
            self.goto('CENTER')
            return

        if x - self.scan_origin > self.scan_max:
            # 스캔 실패 — CCTV target_pose 기반 폴백 (앞 축 = 차량중심 +0.125)
            self.stop()
            self.wheel_x = self.target_xy[0] + 0.125
            self.get_logger().warn(
                f'초음파 스캔 실패 — CCTV 폴백 사용 x={self.wheel_x:.3f}')
            self.goto('CENTER')
            return

        self.creep(self.creep_v)

    def st_center(self):
        self.hold_sync(False)
        dx = self.wheel_x - self.front[0]
        if abs(dx) < self.center_tol:
            self.stop()
            self.get_logger().info(
                f'축 정렬 완료 (잔차 {dx * 1000:.1f}mm) — 그리퍼 전개')
            self.goto('GRIP')
            return
        v = max(-self.creep_v, min(self.creep_v, 1.5 * dx))
        # 너무 느려지지 않게 최소 속도 보장
        if abs(v) < 0.004:
            v = math.copysign(0.004, dx)
        self.creep(v)

    def st_grip(self):
        self.hold_sync(False)
        self.stop()
        for p in self.pub_grips:
            p.publish(Float64(data=self.grip_angle))
        if self.elapsed() > self.grip_settle:
            self.get_logger().info('그리퍼 전개 완료 (180°→90°) — 차량 결합')
            self.goto('ATTACH')

    def st_attach(self):
        self.hold_sync(False)
        self.stop()
        for p in self.pub_attach:
            p.publish(Empty())
        if self.elapsed() > 1.0:
            self.lifted = True
            self.get_logger().info(
                '*** 차량 결합 완료 — /robot/lifted 발행, 운반 주행 허가 ***')
            self.goto('DONE')

    def st_done(self):
        self.hold_sync(True)

    def publish_status(self):
        self.pub_lifted.publish(Bool(data=self.lifted))


def main(args=None):
    rclpy.init(args=args)
    node = GripperController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
