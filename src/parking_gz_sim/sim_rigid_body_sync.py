#!/usr/bin/env python3
"""
sim_rigid_body_sync.py — Gazebo 제어 검증판 v2

목적:
  - A* waypoint를 따라 두 로봇의 가상 중심(base_virtual)을 이동
  - Front/Rear 간격(wheelbase) 유지
  - Front/Rear yaw가 강체축 heading에서 크게 틀어지지 않게 보정
  - 도착/비상정지/오차 과대 상태에서 안전하게 정지

중요:
  - 실제 메카넘 바퀴 물리 검증이 아니라 이상적인 holonomic velocity model 검증이다.
  - Gazebo odom이 world pose로 나오는 경우와 spawn 기준 relative pose로 나오는 경우를
    odom_mode=auto로 자동 판별한다.
"""

import math
import json

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Bool, String


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def ang_norm(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


class SimRigidBodySync(Node):
    def __init__(self):
        super().__init__('sim_rigid_body_sync')

        # ===== 기본 파라미터 =====
        self.declare_parameter('wheelbase', 0.25)
        self.declare_parameter('lookahead', 0.45)
        self.declare_parameter('max_speed', 0.07)
        self.declare_parameter('max_omega', 0.35)
        self.declare_parameter('goal_tolerance', 0.04)
        self.declare_parameter('slow_radius', 0.60)

        # 경로 추종 / 동기 보정
        self.declare_parameter('kp_yaw_path', 0.60)
        self.declare_parameter('kp_dist_sync', 0.85)
        self.declare_parameter('kd_dist_sync', 0.10)
        self.declare_parameter('kp_yaw_sync', 0.65)
        self.declare_parameter('max_sync_corr', 0.045)
        self.declare_parameter('max_yaw_corr', 0.18)
        self.declare_parameter('odom_timeout', 0.5)

        # Fail-safe 기준
        self.declare_parameter('dist_warn', 0.03)       # 30mm
        self.declare_parameter('dist_stop', 0.08)       # 80mm
        self.declare_parameter('yaw_warn_deg', 8.0)
        self.declare_parameter('yaw_stop_deg', 18.0)
        self.declare_parameter('axis_yaw_stop_deg', 25.0)

        # Gazebo VelocityControl 입력 프레임
        # robot: world velocity를 각 로봇 body frame으로 변환해서 Twist 발행
        # world: world velocity를 그대로 Twist 발행
        self.declare_parameter('command_frame', 'robot')

        # odom_mode:
        #   auto     : 첫 odom 샘플로 world/relative 자동 판별
        #   world    : odom pose를 world 좌표로 그대로 사용
        #   relative : odom pose에 spawn 초기 좌표를 더해서 world 좌표 복원
        self.declare_parameter('odom_mode', 'auto')
        self.declare_parameter('front_init_x', 0.75)
        self.declare_parameter('front_init_y', 0.60)
        self.declare_parameter('rear_init_x', 0.50)
        self.declare_parameter('rear_init_y', 0.60)

        gp = self.get_parameter
        self.L = float(gp('wheelbase').value)
        self.half_L = self.L / 2.0
        self.lookahead = float(gp('lookahead').value)
        self.max_v = float(gp('max_speed').value)
        self.max_w = float(gp('max_omega').value)
        self.goal_tol = float(gp('goal_tolerance').value)
        self.slow_r = float(gp('slow_radius').value)
        self.kp_path = float(gp('kp_yaw_path').value)
        self.kp_dist = float(gp('kp_dist_sync').value)
        self.kd_dist = float(gp('kd_dist_sync').value)
        self.kp_ysync = float(gp('kp_yaw_sync').value)
        self.max_sync_corr = float(gp('max_sync_corr').value)
        self.max_yaw_corr = float(gp('max_yaw_corr').value)
        self.odom_to = float(gp('odom_timeout').value)
        self.dist_warn = float(gp('dist_warn').value)
        self.dist_stop = float(gp('dist_stop').value)
        self.yaw_warn = math.radians(float(gp('yaw_warn_deg').value))
        self.yaw_stop = math.radians(float(gp('yaw_stop_deg').value))
        self.axis_yaw_stop = math.radians(float(gp('axis_yaw_stop_deg').value))
        self.command_frame = str(gp('command_frame').value).lower()
        self.odom_mode_param = str(gp('odom_mode').value).lower()
        self.odom_mode = None if self.odom_mode_param == 'auto' else self.odom_mode_param

        # ===== 상태 =====
        self.wps = []
        self.wp_idx = 0
        self.goal_reached = False
        self.estop = False

        self.front_raw = None  # raw odom pose: (x, y, yaw)
        self.rear_raw = None
        self.front = None      # world pose after odom mode conversion
        self.rear = None
        self.t_front = None
        self.t_rear = None

        self.prev_dist_err = 0.0
        self.prev_dist_t = None
        self._err = {}

        # ===== 통신 =====
        self.create_subscription(Path, '/virtual_robot/waypoints', self.path_cb, 10)
        self.create_subscription(Odometry, '/front/odom', lambda m: self.odom_cb('front', m), 20)
        self.create_subscription(Odometry, '/rear/odom', lambda m: self.odom_cb('rear', m), 20)
        self.create_subscription(Bool, '/emergency_stop', self.estop_cb, 10)

        self.pub_f = self.create_publisher(Twist, '/front/cmd_vel', 10)
        self.pub_r = self.create_publisher(Twist, '/rear/cmd_vel', 10)
        self.pub_err = self.create_publisher(String, '/sync/error_state', 10)
        self.pub_done = self.create_publisher(Bool, '/sync/goal_reached', 10)

        self.create_timer(0.02, self.control_loop)
        self.create_timer(0.2, self.publish_error)

        self.get_logger().info(
            f'sim_rigid_body_sync v2 시작: command_frame={self.command_frame}, odom_mode={self.odom_mode_param}')

    # ================= 콜백 =================
    def path_cb(self, msg):
        new = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        if new and new != self.wps:
            self.wps = new
            self.wp_idx = 0
            self.goal_reached = False
            self.prev_dist_t = None
            self.prev_dist_err = 0.0
            self.get_logger().info(f'waypoint {len(new)}개 수신 — 주행 시작')

    def odom_cb(self, role, msg):
        p = msg.pose.pose.position
        yaw = yaw_from_quat(msg.pose.pose.orientation)
        raw = (float(p.x), float(p.y), float(yaw))

        if role == 'front':
            self.front_raw = raw
            self.t_front = self.now_s()
        else:
            self.rear_raw = raw
            self.t_rear = self.now_s()

        self.update_odom_mode_if_needed()
        self.convert_raw_odom()

    def estop_cb(self, msg):
        self.estop = bool(msg.data)
        if self.estop:
            self.get_logger().warn('비상정지!')

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # ================= odom 처리 =================
    def update_odom_mode_if_needed(self):
        if self.odom_mode is not None:
            return
        if self.front_raw is None or self.rear_raw is None:
            return

        fx, fy, _ = self.front_raw
        rx, ry, _ = self.rear_raw
        raw_dist = math.hypot(fx - rx, fy - ry)

        # Gazebo odom이 world pose면 첫 샘플에서 front/rear 간격이 wheelbase 근처다.
        # relative pose면 두 로봇 모두 0,0 근처라 raw_dist가 매우 작다.
        if abs(raw_dist - self.L) < 0.08:
            self.odom_mode = 'world'
        else:
            self.odom_mode = 'relative'

        self.get_logger().warn(
            f'odom_mode 자동판별: raw_dist={raw_dist:.3f}m, wheelbase={self.L:.3f}m → {self.odom_mode}')

    def convert_raw_odom(self):
        if self.front_raw is None or self.rear_raw is None or self.odom_mode is None:
            return

        gp = self.get_parameter
        fx, fy, fth = self.front_raw
        rx, ry, rth = self.rear_raw

        if self.odom_mode == 'relative':
            fx += float(gp('front_init_x').value)
            fy += float(gp('front_init_y').value)
            rx += float(gp('rear_init_x').value)
            ry += float(gp('rear_init_y').value)
        elif self.odom_mode != 'world':
            self.get_logger().error(f'지원하지 않는 odom_mode: {self.odom_mode}')
            self.odom_mode = 'world'

        self.front = (fx, fy, fth)
        self.rear = (rx, ry, rth)

    # ================= 유틸 =================
    def clamp(self, x, lo, hi):
        return max(lo, min(hi, x))

    def limit_xy(self, vx, vy, limit=None):
        lim = self.max_v if limit is None else limit
        mag = math.hypot(vx, vy)
        if mag > lim and mag > 1e-9:
            scale = lim / mag
            return vx * scale, vy * scale
        return vx, vy

    def world_to_robot_velocity(self, vx_w, vy_w, robot_yaw):
        c, s = math.cos(robot_yaw), math.sin(robot_yaw)
        vx_r = c * vx_w + s * vy_w
        vy_r = -s * vx_w + c * vy_w
        return vx_r, vy_r

    def make_cmd(self, vx_w, vy_w, wz, robot_yaw, speed_limit=None):
        if self.command_frame == 'world':
            vx, vy = vx_w, vy_w
        else:
            vx, vy = self.world_to_robot_velocity(vx_w, vy_w, robot_yaw)

        vx, vy = self.limit_xy(vx, vy, speed_limit)
        wz = self.clamp(wz, -self.max_w, self.max_w)
        return [vx, vy, wz]

    # ================= 메인 루프 =================
    def control_loop(self):
        if self.estop or self.goal_reached or not self.wps or self.front is None or self.rear is None:
            self.send_stop()
            return

        now = self.now_s()
        if self.t_front is None or self.t_rear is None:
            self.send_stop()
            return
        if now - self.t_front > self.odom_to or now - self.t_rear > self.odom_to:
            self._err['stop_reason'] = 'odom_timeout'
            self.send_stop()
            return

        fx, fy, fth = self.front
        rx, ry, rth = self.rear

        # ---- 가상 강체 중심 ----
        # description: front/rear pose를 평균내서 강체 중심 pose를 계산
        cx, cy = (fx + rx) / 2.0, (fy + ry) / 2.0
        dx_fr, dy_fr = fx - rx, fy - ry
        dist = math.hypot(dx_fr, dy_fr)
        if dist > 1e-9:
            axis_x, axis_y = dx_fr / dist, dy_fr / dist
        else:
            axis_x, axis_y = math.cos(fth), math.sin(fth)
        heading = math.atan2(axis_y, axis_x)

        dist_err = dist - self.L
        yaw_err = ang_norm(fth - rth)
        f_axis_err = ang_norm(fth - heading)
        r_axis_err = ang_norm(rth - heading)
        max_axis_err = max(abs(f_axis_err), abs(r_axis_err))

        # ---- 오차 과대 fail-safe ----
        hard_stop = False
        stop_reason = ''
        if abs(dist_err) > self.dist_stop:
            hard_stop = True
            stop_reason = 'dist_error_stop'
        elif abs(yaw_err) > self.yaw_stop:
            hard_stop = True
            stop_reason = 'front_rear_yaw_stop'
        elif max_axis_err > self.axis_yaw_stop:
            hard_stop = True
            stop_reason = 'axis_yaw_stop'

        if hard_stop:
            self._err = self.make_error_dict(dist_err, yaw_err, f_axis_err, r_axis_err, cx, cy, 0.0, stop_reason)
            self.send_stop()
            return

        # ---- 도착 판정: 위치만 보지 않고 동기 상태까지 확인 ----
        gx, gy = self.wps[-1]
        d_goal = math.hypot(gx - cx, gy - cy)
        if d_goal < self.goal_tol and abs(dist_err) < self.dist_warn and abs(yaw_err) < self.yaw_warn:
            self.goal_reached = True
            self.send_stop()
            self.pub_done.publish(Bool(data=True))
            self.get_logger().info(
                f'*** 목표 도착 *** pos={d_goal*100:.1f}cm, dist_err={dist_err*1000:.1f}mm, yaw={math.degrees(yaw_err):.1f}deg')
            return

        # ---- Pure Pursuit target 선택 ----
        while self.wp_idx < len(self.wps) - 1 and \
                math.hypot(self.wps[self.wp_idx][0] - cx,
                           self.wps[self.wp_idx][1] - cy) < self.lookahead:
            self.wp_idx += 1
        tx, ty = self.wps[self.wp_idx]
        dx, dy = tx - cx, ty - cy
        d = math.hypot(dx, dy)
        if d < 1e-6:
            self.send_stop()
            return

        # ---- 동기 오차가 커질수록 중심 이동속도를 낮춤 ----
        speed_scale = 1.0
        if abs(dist_err) > self.dist_warn:
            speed_scale *= 0.35
        if abs(yaw_err) > self.yaw_warn or max_axis_err > self.yaw_warn:
            speed_scale *= 0.50

        speed = self.max_v
        if d_goal < self.slow_r:
            speed = max(0.02, self.max_v * d_goal / self.slow_r)
        speed *= speed_scale

        vwx, vwy = dx / d * speed, dy / d * speed

        # 경로 방향으로 강체축 정렬
        path_yaw = math.atan2(dy, dx)
        path_err = ang_norm(path_yaw - heading)
        omega = self.clamp(self.kp_path * path_err, -self.max_w, self.max_w)

        # ---- 강체 기구학 속도 분배 ----
        f_vx_w = vwx - omega * self.half_L * axis_y
        f_vy_w = vwy + omega * self.half_L * axis_x
        r_vx_w = vwx + omega * self.half_L * axis_y
        r_vy_w = vwy - omega * self.half_L * axis_x

        # ---- 거리 동기 보정: PD spring-damper ----
        dt = 0.02 if self.prev_dist_t is None else max(1e-3, now - self.prev_dist_t)
        d_dist_err = (dist_err - self.prev_dist_err) / dt
        self.prev_dist_err = dist_err
        self.prev_dist_t = now

        sync_corr = self.kp_dist * dist_err + self.kd_dist * d_dist_err
        sync_corr = self.clamp(sync_corr, -self.max_sync_corr, self.max_sync_corr)

        # 간격이 벌어지면 front를 뒤로, rear를 앞으로 당긴다.
        f_vx_w -= 0.5 * sync_corr * axis_x
        f_vy_w -= 0.5 * sync_corr * axis_y
        r_vx_w += 0.5 * sync_corr * axis_x
        r_vy_w += 0.5 * sync_corr * axis_y

        # ---- yaw 동기 보정 ----
        f_yaw_corr = self.clamp(self.kp_ysync * f_axis_err, -self.max_yaw_corr, self.max_yaw_corr)
        r_yaw_corr = self.clamp(self.kp_ysync * r_axis_err, -self.max_yaw_corr, self.max_yaw_corr)
        f_w = omega - f_yaw_corr
        r_w = omega - r_yaw_corr

        # ---- 최종 cmd_vel ----
        f_cmd = self.make_cmd(f_vx_w, f_vy_w, f_w, fth, speed_limit=self.max_v)
        r_cmd = self.make_cmd(r_vx_w, r_vy_w, r_w, rth, speed_limit=self.max_v)
        self.publish_cmd(self.pub_f, f_cmd)
        self.publish_cmd(self.pub_r, r_cmd)

        self._err = self.make_error_dict(dist_err, yaw_err, f_axis_err, r_axis_err, cx, cy,
                                         sync_corr, 'ok', d_goal=d_goal,
                                         speed_scale=speed_scale, wp=f'{self.wp_idx + 1}/{len(self.wps)}')

    def make_error_dict(self, dist_err, yaw_err, f_axis_err, r_axis_err, cx, cy, sync_corr,
                        status, d_goal=None, speed_scale=None, wp=None):
        data = {
            'status': status,
            'odom_mode': self.odom_mode or 'unknown',
            'cmd_frame': self.command_frame,
            'center_x': round(cx, 3),
            'center_y': round(cy, 3),
            'dist_err_mm': round(dist_err * 1000, 1),
            'yaw_err_deg': round(math.degrees(yaw_err), 2),
            'front_axis_err_deg': round(math.degrees(f_axis_err), 2),
            'rear_axis_err_deg': round(math.degrees(r_axis_err), 2),
            'sync_corr_cm_s': round(sync_corr * 100, 2),
        }
        if d_goal is not None:
            data['goal_dist_cm'] = round(d_goal * 100, 1)
        if speed_scale is not None:
            data['speed_scale'] = round(speed_scale, 2)
        if wp is not None:
            data['wp'] = wp
        return data

    def publish_cmd(self, pub, cmd):
        t = Twist()
        t.linear.x = float(cmd[0])
        t.linear.y = float(cmd[1])
        t.angular.z = float(cmd[2])
        pub.publish(t)

    def send_stop(self):
        z = Twist()
        self.pub_f.publish(z)
        self.pub_r.publish(z)

    def publish_error(self):
        if not self._err:
            return
        m = String()
        m.data = json.dumps(self._err, ensure_ascii=False)
        self.pub_err.publish(m)

        status = self._err.get('status', 'ok')
        if status != 'ok':
            self.get_logger().warn(f'sync status={status}: {m.data}')
        elif abs(self._err.get('dist_err_mm', 0.0)) > self.dist_warn * 1000:
            self.get_logger().warn(f"거리 오차 경고: {self._err['dist_err_mm']}mm")


def main(args=None):
    rclpy.init(args=args)
    node = SimRigidBodySync()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.send_stop()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
