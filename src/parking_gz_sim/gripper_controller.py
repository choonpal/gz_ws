#!/usr/bin/env python3
"""
gripper_controller.py — 차량 밑 삽입 + 초음파 바퀴 감지 + 그리퍼 파지 + 결합

흐름:
  INIT_DETACH : DetachableJoint가 attach 상태로 스폰되므로 기동 직후 분리
  WAIT        : 플래너가 차량 +x쪽 정렬점(standoff)까지 안내.
                /sync/goal_reached + 정렬점 근접 → 시퀀스 시작
                (이때부터 /sync/enable=False로 주행 제어권을 가져옴)
  INSERT      : -x 저속 직진으로 차량 밑 삽입. 차량 축선(target_y)에
                y 서보, yaw는 x축 정렬 유지 — 바퀴 안쪽면과의 여유는
                좌우 4.5cm뿐이므로 정렬이 생명.
                삽입 중 각 로봇이 자기 초음파로 "값이 바뀌는 지점"(에지)을
                기록: 에코 시작/끝의 중점 = 바퀴 축 x.
                front 로봇은 앞 축(x > 차량중심)만, rear 로봇은
                뒤 축(x < 차량중심)만 인정 — rear가 앞 축을 지나며 받는
                에코는 게이트로 걸러낸다.
  CENTER      : 각 로봇을 자기 바퀴 축 x로 독립 P-제어 정렬
  GRIP        : 그리퍼 4개 180°(접힘) → 90°(전개) — 바퀴 바깥쪽 케이지
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


def ang_norm(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def clamp(v, lim):
    return max(-lim, min(lim, v))


class GripperController(Node):
    ROLES = ('front', 'rear')

    def __init__(self):
        super().__init__('gripper_controller')

        # 초음파 에코 판정 임계 [m] — 정렬 시 바퀴 안쪽면까지 ~5.5cm
        self.declare_parameter('echo_threshold', 0.10)
        # 정렬점: 차량 중심 +x 거리 [m] — map_astar_planner와 동일해야 함
        self.declare_parameter('approach_standoff', 0.60)
        # 삽입 직진 속도 [m/s] (20Hz 초음파 기준 샘플당 ~1.3mm)
        self.declare_parameter('insert_speed', 0.025)
        self.declare_parameter('creep_speed', 0.012)
        # 축 정렬 허용 오차 [m] — 그리퍼 케이지 여유(2.5~3.5cm)보다
        # 작으면 충분. 너무 빡빡하면 odom 지터로 수렴 못 하고 정체된다.
        self.declare_parameter('center_tol', 0.010)
        # CENTER 수렴 제한 시간 [s] — 초과 시 잔차 경고 후 GRIP 진행
        self.declare_parameter('center_timeout', 20.0)
        # 축선 y 서보 / yaw 유지 게인 (좌우 여유 4.5cm 확보용)
        self.declare_parameter('y_gain', 1.2)
        self.declare_parameter('max_vy', 0.02)
        self.declare_parameter('yaw_gain', 1.5)
        self.declare_parameter('max_wz', 0.15)
        # 차량 축간거리의 절반 [m] — 폴백·게이트 기준
        self.declare_parameter('wheel_half_base', 0.125)
        # 그리퍼 전개 각도 [rad] = 90°
        self.declare_parameter('grip_angle', 1.5708)
        self.declare_parameter('grip_settle_time', 3.0)
        # 정렬점 도착 인정 반경 [m]
        self.declare_parameter('start_tol', 0.25)

        gp = self.get_parameter
        self.echo_th = float(gp('echo_threshold').value)
        self.standoff = float(gp('approach_standoff').value)
        self.insert_v = float(gp('insert_speed').value)
        self.creep_v = float(gp('creep_speed').value)
        self.center_tol = float(gp('center_tol').value)
        self.center_timeout = float(gp('center_timeout').value)
        self.y_gain = float(gp('y_gain').value)
        self.max_vy = float(gp('max_vy').value)
        self.yaw_gain = float(gp('yaw_gain').value)
        self.max_wz = float(gp('max_wz').value)
        self.whb = float(gp('wheel_half_base').value)
        self.grip_angle = float(gp('grip_angle').value)
        self.grip_settle = float(gp('grip_settle_time').value)
        self.start_tol = float(gp('start_tol').value)

        # ===== 상태 =====
        self.state = 'INIT_DETACH'
        self.state_t0 = None
        self.pose = {'front': None, 'rear': None}   # (x, y, yaw) world
        self.target_xy = None
        self.goal_reached = False
        self.lifted = False
        # 초음파 최신값 [role][L/R], 에지 기록, 확정 축 x
        self.us = {r: {'L': 99.0, 'R': 99.0} for r in self.ROLES}
        self.edge = None
        self.wheel_x = {r: None for r in self.ROLES}
        # 삽입 시작 시 고정하는 차량 축 좌표 — CCTV 픽셀 지터를
        # y 서보가 계속 쫓지 않도록 latch
        self.axis_xy = None
        self.last_dbg = 0.0

        # ===== 통신 =====
        for role in self.ROLES:
            self.create_subscription(
                Odometry, f'/{role}/odom',
                lambda m, r=role: self.odom_cb(r, m), 20)
            for side, name in (('L', 'us_left'), ('R', 'us_right')):
                self.create_subscription(
                    LaserScan, f'/{role}/{name}',
                    lambda m, r=role, s=side: self.us_cb(r, s, m), 10)
        self.create_subscription(Bool, '/sync/goal_reached',
                                 self.reached_cb, 10)
        self.create_subscription(PoseStamped, '/parking/target_pose',
                                 self.target_cb, 10)

        self.pub_cmd = {r: self.create_publisher(Twist, f'/{r}/cmd_vel', 10)
                        for r in self.ROLES}
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
        self.pose[role] = (float(p.x), float(p.y), yaw)

    def us_cb(self, role, side, msg):
        """HC-SR04 대역: 스캔 전체의 min 하나만 단일 거리값으로 사용."""
        valid = [r for r in msg.ranges
                 if msg.range_min <= r <= msg.range_max]
        self.us[role][side] = min(valid) if valid else 99.0

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

    def cmd_robot(self, role, vx_world, servo=True):
        """world x 속도 + 축선 y 서보 + yaw 유지 → body frame 변환 발행."""
        t = Twist()
        pose = self.pose[role]
        if pose is not None:
            x, y, yaw = pose
            vy_world = 0.0
            axis = self.axis_xy or self.target_xy
            if servo and axis is not None:
                vy_world = clamp(
                    self.y_gain * (axis[1] - y), self.max_vy)
            # x축 정렬 유지 (0 또는 π 중 가까운 쪽 — 뒤집힌 자세도 허용)
            yaw_err = ang_norm(yaw)
            if abs(yaw_err) > math.pi / 2:
                yaw_err = ang_norm(yaw - math.pi)
            t.linear.x = math.cos(yaw) * vx_world + math.sin(yaw) * vy_world
            t.linear.y = -math.sin(yaw) * vx_world + math.cos(yaw) * vy_world
            t.angular.z = clamp(-self.yaw_gain * yaw_err, self.max_wz)
        self.pub_cmd[role].publish(t)

    def stop(self):
        for role in self.ROLES:
            self.pub_cmd[role].publish(Twist())

    def hold_sync(self, enabled):
        self.pub_enable.publish(Bool(data=enabled))

    def expected_axle_x(self, role):
        """CCTV 기준 예상 축 위치 — front는 앞 축, rear는 뒤 축."""
        sign = 1.0 if role == 'front' else -1.0
        return self.axis_xy[0] + sign * self.whb

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
            self.get_logger().info('초기 분리 완료 — 정렬점 도착 대기')
            self.goto('WAIT')

    def st_wait(self):
        if not self.goal_reached or self.lifted:
            return
        if any(self.pose[r] is None for r in self.ROLES) \
                or self.target_xy is None:
            return
        cx = (self.pose['front'][0] + self.pose['rear'][0]) / 2
        cy = (self.pose['front'][1] + self.pose['rear'][1]) / 2
        sx = self.target_xy[0] + self.standoff
        sy = self.target_xy[1]
        d = math.hypot(cx - sx, cy - sy)
        if d > self.start_tol:
            # 최종 목표 도착 등 다른 goal_reached — 무시
            self.goal_reached = False
            return
        self.get_logger().info(
            f'정렬점 도착 확인 (오차 {d * 100:.1f}cm) — 차량 밑 삽입 시작')
        # 차량은 정차 확정 상태 — 이후 CCTV 지터를 쫓지 않도록 축 고정
        self.axis_xy = (self.target_xy[0], self.target_xy[1])
        self.edge = {r: {s: {'in': False, 'enter': None, 'center': None}
                         for s in ('L', 'R')} for r in self.ROLES}
        self.goto('INSERT')

    def st_insert(self):
        self.hold_sync(False)
        tx = self.axis_xy[0]

        for role in self.ROLES:
            x = self.pose[role][0]
            # 자기 축 구간에서만 에코 인정 — rear가 앞 축을 지날 때 무시
            gate = (x > tx) if role == 'front' else (x < tx)

            for s in ('L', 'R'):
                e = self.edge[role][s]
                if e['center'] is not None:
                    continue
                echo = self.us[role][s] < self.echo_th
                if echo and gate and not e['in']:
                    e['in'] = True
                    e['enter'] = x
                    self.get_logger().info(
                        f'초음파[{role}/{s}] 에코 시작 x={x:.3f} '
                        f'(거리 {self.us[role][s]:.3f}m)')
                elif e['in'] and (not echo or not gate):
                    e['center'] = (e['enter'] + x) / 2
                    self.get_logger().info(
                        f'초음파[{role}/{s}] 에코 종료 x={x:.3f} '
                        f'→ 바퀴 중심 {e["center"]:.3f}')

            if self.wheel_x[role] is None:
                centers = [self.edge[role][s]['center'] for s in ('L', 'R')
                           if self.edge[role][s]['center'] is not None]
                if len(centers) == 2:
                    self.wheel_x[role] = sum(centers) / 2
                    self.get_logger().info(
                        f'{role} 바퀴 축 확정: x={self.wheel_x[role]:.3f}')
                elif x < self.expected_axle_x(role) - 0.12:
                    # 예상 축을 12cm 지나도록 미검출 — CCTV 폴백
                    self.wheel_x[role] = self.expected_axle_x(role)
                    self.get_logger().warn(
                        f'{role} 초음파 미검출 — CCTV 폴백 '
                        f'x={self.wheel_x[role]:.3f}')

        if all(self.wheel_x[r] is not None for r in self.ROLES):
            self.stop()
            self.get_logger().info('양 축 확정 — 개별 정렬 시작')
            self.goto('CENTER')
            return

        for role in self.ROLES:
            self.cmd_robot(role, -self.insert_v)

    def st_center(self):
        self.hold_sync(False)
        done = True
        for role in self.ROLES:
            dx = self.wheel_x[role] - self.pose[role][0]
            if abs(dx) < self.center_tol:
                self.cmd_robot(role, 0.0)
                continue
            done = False
            v = clamp(1.5 * dx, self.creep_v)
            # 너무 느려지지 않게 최소 속도 보장
            if abs(v) < 0.004:
                v = math.copysign(0.004, dx)
            self.cmd_robot(role, v)
        # 수렴 정체 진단용 잔차 로그 (2초마다)
        if self.now_s() - self.last_dbg > 2.0:
            self.last_dbg = self.now_s()
            fr = self.wheel_x['front'] - self.pose['front'][0]
            rr = self.wheel_x['rear'] - self.pose['rear'][0]
            self.get_logger().info(
                f'CENTER 잔차 front {fr * 1000:.1f}mm / rear {rr * 1000:.1f}mm')
        if not done and self.elapsed() > self.center_timeout:
            fr = self.wheel_x['front'] - self.pose['front'][0]
            rr = self.wheel_x['rear'] - self.pose['rear'][0]
            self.get_logger().warn(
                f'CENTER 시간 초과 — 잔차 front {fr * 1000:.1f}mm / '
                f'rear {rr * 1000:.1f}mm 상태로 그리퍼 전개 진행')
            done = True
        if done:
            self.stop()
            fr = self.wheel_x['front'] - self.pose['front'][0]
            rr = self.wheel_x['rear'] - self.pose['rear'][0]
            self.get_logger().info(
                f'축 정렬 완료 (잔차 front {fr * 1000:.1f}mm / '
                f'rear {rr * 1000:.1f}mm) — 그리퍼 전개')
            self.goto('GRIP')

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
