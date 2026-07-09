#!/usr/bin/env python3
"""
cctv_map_builder.py — Gazebo 천장 CCTV 이미지 → OccupancyGrid 맵 + 차량 인지

현재 parking_lot.sdf 기준:
  - 주차장 좌표: x 0~6m, y 0~4m
  - CCTV 위치: (3, 2, 4.0)
  - CCTV HFOV: 1.5708 rad
  - CCTV 토픽: /cctv/image
  - 출력 맵 토픽: /parking/map

처리 흐름:
  /cctv/image
  → 빨간색 차량 픽셀 분리 (실전 YOLO 'vehicle' 클래스의 시뮬 대역)
  → 채도 기반 장애물 후보 검출 (차량 픽셀 제외)
  → 픽셀 좌표를 월드 좌표로 변환
  → front/rear robot 위치 주변 self-mask + target 차량 mask (운반 대상 ≠ 장애물)
  → nav_msgs/OccupancyGrid 발행
  → 대기공간 ROI 안 차량이 stop_time 동안 정지하면
    /parking/target_ready = True (리프트 허가, latch)
    이후 /parking/target_pose 지속 발행 (실전 vehicle_pose_feedback 대응)

실전 대응 관계:
  빨간색 분리          ↔ YOLO11n-seg vehicle 클래스
  정차 판정(2cm/2s)    ↔ 동일 로직 (bbox 지터 흡수)
  target 차량 맵 마스킹 ↔ 운반 차량은 장애물이 아님 (vehicle_pose_feedback로 추적)
"""

import math
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool


class CctvMapBuilder(Node):
    def __init__(self):
        super().__init__('cctv_map_builder')

        # =====================================================
        # Camera parameters — parking_lot.sdf와 일치해야 함
        # =====================================================
        self.declare_parameter('cam_x', 3.0)
        self.declare_parameter('cam_y', 2.0)
        self.declare_parameter('cam_z', 4.0)
        self.declare_parameter('hfov', 1.5708)

        # =====================================================
        # Map parameters — 현재 월드 x 0~6, y 0~4
        # =====================================================
        self.declare_parameter('origin_x', 0.0)
        self.declare_parameter('origin_y', 0.0)
        self.declare_parameter('map_w_m', 6.0)
        self.declare_parameter('map_h_m', 4.0)
        self.declare_parameter('resolution', 0.05)

        # =====================================================
        # Segmentation / masking parameters
        # =====================================================
        # RGB max-min 값이 이보다 크면 "유채색 물체"로 판단
        self.declare_parameter('sat_threshold', 40)

        # 로봇 자신을 장애물로 찍지 않기 위한 마스킹 반경 [m]
        self.declare_parameter('robot_mask_radius', 0.25)

        # =====================================================
        # Vehicle detection parameters (차량 인지 파트)
        # =====================================================
        # 대기공간 ROI [x1, y1, x2, y2] — 이 안에서 정차해야 target 인정
        self.declare_parameter('waiting_zone', [0.0, 0.0, 1.3, 1.2])
        # 빨간 차량 판별 임계 (렌더링 음영 고려해 여유 있게)
        self.declare_parameter('veh_r_min', 120)
        self.declare_parameter('veh_g_max', 80)
        self.declare_parameter('veh_b_max', 80)
        # 노이즈 배제: 최소 차량 픽셀 수
        self.declare_parameter('min_vehicle_px', 30)
        # 정차 판정: 이동량 < stop_eps [m] 가 stop_time [s] 연속
        self.declare_parameter('stop_eps', 0.02)
        self.declare_parameter('stop_time', 2.0)
        # target 차량 맵 마스킹 반경 [m] (차체 절반 + 여유)
        self.declare_parameter('vehicle_mask_radius', 0.30)
        # 차량 윗면 높이 [m] — 차체 바닥 0.13 + 두께 0.05 = 0.18.
        # CCTV가 보는 건 윗면이라 지면 스케일로 투영하면 카메라에서
        # 먼 쪽으로 밀린다 (±5cm 요구 초과). 윗면 깊이로 보정.
        self.declare_parameter('vehicle_top_z', 0.18)

        # Topic names
        self.declare_parameter('image_topic', '/cctv/image')
        self.declare_parameter('front_odom_topic', '/front/odom')
        self.declare_parameter('rear_odom_topic', '/rear/odom')
        self.declare_parameter('map_topic', '/parking/map')
        self.declare_parameter('target_pose_topic', '/parking/target_pose')
        self.declare_parameter('target_ready_topic', '/parking/target_ready')

        gp = self.get_parameter

        self.cam_x = float(gp('cam_x').value)
        self.cam_y = float(gp('cam_y').value)
        self.cam_z = float(gp('cam_z').value)
        self.hfov = float(gp('hfov').value)

        self.ox = float(gp('origin_x').value)
        self.oy = float(gp('origin_y').value)
        self.map_w_m = float(gp('map_w_m').value)
        self.map_h_m = float(gp('map_h_m').value)
        self.res = float(gp('resolution').value)

        self.gw = int(round(self.map_w_m / self.res))
        self.gh = int(round(self.map_h_m / self.res))

        self.sat_th = int(gp('sat_threshold').value)
        self.mask_r = float(gp('robot_mask_radius').value)

        self.waiting_zone = [float(v) for v in gp('waiting_zone').value]
        self.veh_r_min = int(gp('veh_r_min').value)
        self.veh_g_max = int(gp('veh_g_max').value)
        self.veh_b_max = int(gp('veh_b_max').value)
        self.min_vehicle_px = int(gp('min_vehicle_px').value)
        self.stop_eps = float(gp('stop_eps').value)
        self.stop_time = float(gp('stop_time').value)
        self.veh_mask_r = float(gp('vehicle_mask_radius').value)
        self.veh_top_z = float(gp('vehicle_top_z').value)

        self.image_topic = str(gp('image_topic').value)
        self.front_odom_topic = str(gp('front_odom_topic').value)
        self.rear_odom_topic = str(gp('rear_odom_topic').value)
        self.map_topic = str(gp('map_topic').value)
        self.target_pose_topic = str(gp('target_pose_topic').value)
        self.target_ready_topic = str(gp('target_ready_topic').value)

        self.front_pos = None
        self.rear_pos = None

        # 차량 인지 상태
        self.veh_hist = []          # [(t, x, y)] 정차 판정용 이력
        self.target_ready = False   # 리프트 허가 (latch)
        self.veh_pos = None         # 최신 차량 중심 (월드 좌표)

        self.create_subscription(Image, self.image_topic, self.image_cb, 1)
        self.create_subscription(
            Odometry,
            self.front_odom_topic,
            lambda msg: self.odom_cb('front', msg),
            10,
        )
        self.create_subscription(
            Odometry,
            self.rear_odom_topic,
            lambda msg: self.odom_cb('rear', msg),
            10,
        )

        self.pub_map = self.create_publisher(OccupancyGrid, self.map_topic, 1)
        self.pub_target = self.create_publisher(
            PoseStamped, self.target_pose_topic, 10)
        self.pub_ready = self.create_publisher(
            Bool, self.target_ready_topic, 10)

        self.frame_n = 0

        self.get_logger().info(
            'cctv_map_builder started | '
            f'image={self.image_topic}, map={self.map_topic}, '
            f'cam=({self.cam_x:.2f},{self.cam_y:.2f},{self.cam_z:.2f}), '
            f'map={self.map_w_m:.2f}x{self.map_h_m:.2f}m, res={self.res:.3f} | '
            f'waiting_zone={self.waiting_zone}, '
            f'stop={self.stop_eps * 1000:.0f}mm/{self.stop_time:.1f}s'
        )

    # =====================================================
    # Odometry callback
    # =====================================================
    def odom_cb(self, role: str, msg: Odometry):
        """
        현재 Gazebo/bridge 구조에서는 /front/odom, /rear/odom이
        world/map 좌표로 들어온다고 가정한다.

        그래서 예전 코드처럼 front_init_x, rear_init_x를 더하지 않는다.
        init 값을 또 더하면 로봇 위치가 두 번 더해져 self-mask가 틀어진다.
        """
        p = msg.pose.pose.position

        if role == 'front':
            self.front_pos = (float(p.x), float(p.y))
        else:
            self.rear_pos = (float(p.x), float(p.y))

    # =====================================================
    # 정차 판정
    # =====================================================
    def _update_stationarity(self, t: float, x: float, y: float) -> bool:
        """중심 좌표 이력을 갱신하고, stop_time 동안 stop_eps 이내면 True."""
        self.veh_hist.append((t, x, y))
        # 판정 윈도우 밖 샘플 제거
        self.veh_hist = [
            h for h in self.veh_hist if t - h[0] <= self.stop_time + 0.5
        ]
        # 이력이 판정 시간만큼 쌓이지 않았으면 아직 판단 불가
        if t - self.veh_hist[0][0] < self.stop_time:
            return False
        return all(
            math.hypot(x - hx, y - hy) < self.stop_eps
            for _, hx, hy in self.veh_hist
        )

    def _in_waiting_zone(self, x: float, y: float) -> bool:
        x1, y1, x2, y2 = self.waiting_zone
        return x1 <= x <= x2 and y1 <= y <= y2

    # =====================================================
    # Image callback
    # =====================================================
    def image_cb(self, msg: Image):
        self.frame_n += 1

        # 15Hz 입력이면 2프레임에 1번 처리 → 약 7.5Hz
        if self.frame_n % 2:
            return

        H = int(msg.height)
        W = int(msg.width)

        if H <= 0 or W <= 0:
            self.get_logger().warn('Invalid image size', throttle_duration_sec=5.0)
            return

        if msg.encoding not in ('rgb8', 'bgr8'):
            self.get_logger().warn(
                f'Unsupported image encoding: {msg.encoding}',
                throttle_duration_sec=5.0,
            )
            return

        # ROS Image는 row padding이 있을 수 있으므로 step을 고려
        raw = np.frombuffer(msg.data, dtype=np.uint8)
        step = int(msg.step) if msg.step else W * 3

        try:
            img = raw.reshape(H, step)[:, :W * 3].reshape(H, W, 3)
        except ValueError:
            self.get_logger().warn(
                f'Failed to reshape image: H={H}, W={W}, step={step}, raw={raw.size}',
                throttle_duration_sec=5.0,
            )
            return

        # =====================================================
        # 0) 채널 분리 — 채도 분할은 순서 무관이지만
        #    차량(빨강) 분류는 encoding에 따라 채널 순서가 다름
        # =====================================================
        if msg.encoding == 'bgr8':
            ch_r = img[:, :, 2].astype(np.int16)
            ch_g = img[:, :, 1].astype(np.int16)
            ch_b = img[:, :, 0].astype(np.int16)
        else:  # rgb8
            ch_r = img[:, :, 0].astype(np.int16)
            ch_g = img[:, :, 1].astype(np.int16)
            ch_b = img[:, :, 2].astype(np.int16)

        # =====================================================
        # 1) 차량 픽셀 분류 (실전 YOLO vehicle 클래스의 시뮬 대역)
        # =====================================================
        vehicle_px = (
            (ch_r > self.veh_r_min)
            & (ch_g < self.veh_g_max)
            & (ch_b < self.veh_b_max)
        )

        # =====================================================
        # 2) 장애물 후보 분할: 채도 max(RGB)-min(RGB), 차량 픽셀 제외
        # =====================================================
        imax = img.max(axis=2).astype(np.int16)
        imin = img.min(axis=2).astype(np.int16)
        obstacle = ((imax - imin) > self.sat_th) & (~vehicle_px)

        # =====================================================
        # 3) 픽셀 좌표 → 월드 좌표
        # =====================================================
        # Gazebo overhead camera가 pitch=+1.5708이고 아래를 본다고 가정.
        #
        # 단순 핀홀 근사:
        #   가로 시야 폭 = 2 * cam_z * tan(hfov/2)
        #   s = m/px
        #
        # 이미지 u축/ v축과 월드 x/y 방향은 카메라 회전 설정에 따라 바뀔 수 있다.
        # 현재 SDF pose <pose>3 2 4.0 0 1.5708 0</pose> 기준으로:
        #   u 증가 방향 → world -y
        #   v 증가 방향 → world -x
        #
        # 만약 맵이 좌우/상하 반전이면 아래 wx/wy 부호만 바꾸면 된다.
        s = 2.0 * self.cam_z * math.tan(self.hfov / 2.0) / float(W)

        vs, us = np.nonzero(obstacle)

        wx = self.cam_x - (vs - H / 2.0 + 0.5) * s
        wy = self.cam_y - (us - W / 2.0 + 0.5) * s

        # =====================================================
        # 4) 차량 중심 계산 + 정차 판정 + target 발행
        # =====================================================
        now = self.get_clock().now()
        t = now.nanoseconds * 1e-9

        vvs, vus = np.nonzero(vehicle_px)
        vehicle_seen = vvs.size >= self.min_vehicle_px

        if vehicle_seen:
            # 차량 윗면 깊이(cam_z - vehicle_top_z) 기준 스케일로 투영
            sv = 2.0 * (self.cam_z - self.veh_top_z) \
                * math.tan(self.hfov / 2.0) / float(W)
            vwx = self.cam_x - (vvs - H / 2.0 + 0.5) * sv
            vwy = self.cam_y - (vus - W / 2.0 + 0.5) * sv
            vx = float(vwx.mean())
            vy = float(vwy.mean())
            self.veh_pos = (vx, vy)

            if not self.target_ready:
                # 대기공간 안에서만 정차 판정 (밖에서 정지해도 target 아님)
                if self._in_waiting_zone(vx, vy):
                    if self._update_stationarity(t, vx, vy):
                        self.target_ready = True
                        self.get_logger().info(
                            f'차량 정차 확정 ({vx:.2f}, {vy:.2f}) — '
                            'TARGET_READY, 리프트 허가'
                        )
                else:
                    # ROI 밖 = 아직 진입 중, 이력 리셋
                    self.veh_hist = []

            if self.target_ready:
                # 정차 확정 후 지속 발행 — 실전 vehicle_pose_feedback 대응
                tp = PoseStamped()
                tp.header.stamp = now.to_msg()
                tp.header.frame_id = 'map'
                tp.pose.position.x = vx
                tp.pose.position.y = vy
                tp.pose.orientation.w = 1.0
                self.pub_target.publish(tp)
        else:
            # 미검출: 정차 이력 리셋 (ready는 latch 유지 —
            # 그립 단계에서 로봇에 가려 잠깐 안 보이는 상황 대비)
            self.veh_hist = []

        rd = Bool()
        rd.data = self.target_ready
        self.pub_ready.publish(rd)

        # =====================================================
        # 5) OccupancyGrid 채우기
        # =====================================================
        grid = np.zeros((self.gh, self.gw), dtype=np.int8)

        gx = ((wx - self.ox) / self.res).astype(np.int32)
        gy = ((wy - self.oy) / self.res).astype(np.int32)

        ok = (gx >= 0) & (gx < self.gw) & (gy >= 0) & (gy < self.gh)
        grid[gy[ok], gx[ok]] = 100

        # =====================================================
        # 6) 로봇 자신 + target 차량 마스킹
        #    (차량은 운반 대상이라 장애물이 아님 — 경계 픽셀의
        #     안티앨리어싱이 채도 분할에 잡히는 것도 함께 제거)
        # =====================================================
        mask_list = []
        for pos in (self.front_pos, self.rear_pos):
            if pos is not None:
                mask_list.append((pos, self.mask_r))
        if self.veh_pos is not None:
            mask_list.append((self.veh_pos, self.veh_mask_r))

        for pos, radius in mask_list:
            r = int(round(radius / self.res))
            cx = int(round((pos[0] - self.ox) / self.res))
            cy = int(round((pos[1] - self.oy) / self.res))

            y0 = max(0, cy - r)
            y1 = min(self.gh, cy + r + 1)
            x0 = max(0, cx - r)
            x1 = min(self.gw, cx + r + 1)

            grid[y0:y1, x0:x1] = 0

        # =====================================================
        # 7) OccupancyGrid 발행
        # =====================================================
        m = OccupancyGrid()
        m.header.stamp = now.to_msg()
        m.header.frame_id = 'map'

        m.info.resolution = self.res
        m.info.width = self.gw
        m.info.height = self.gh
        m.info.origin.position.x = self.ox
        m.info.origin.position.y = self.oy
        m.info.origin.position.z = 0.0
        m.info.origin.orientation.w = 1.0

        m.data = grid.flatten().tolist()

        self.pub_map.publish(m)

        occ = int((grid > 0).sum())
        veh_str = (
            f'({self.veh_pos[0]:.2f},{self.veh_pos[1]:.2f})'
            if vehicle_seen else 'none'
        )
        self.get_logger().info(
            f'published {self.map_topic} | occupied_cells={occ} | '
            f'vehicle={veh_str} ready={self.target_ready}',
            throttle_duration_sec=5.0,
        )


def main(args=None):
    rclpy.init(args=args)
    node = CctvMapBuilder()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()