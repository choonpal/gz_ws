#!/usr/bin/env python3
"""
cctv_map_builder.py — Gazebo 천장 CCTV 이미지 → OccupancyGrid 맵 생성

현재 parking_lot.sdf 기준:
  - 주차장 좌표: x 0~6m, y 0~4m
  - CCTV 위치: (3, 2, 4.0)
  - CCTV HFOV: 1.5708 rad
  - CCTV 토픽: /cctv/image
  - 출력 맵 토픽: /parking/map

처리 흐름:
  /cctv/image
  → 채도 기반 장애물 후보 검출
  → 픽셀 좌표를 월드 좌표로 변환
  → front/rear robot 위치 주변은 self-mask
  → nav_msgs/OccupancyGrid 발행

주의:
  지금 버전은 시뮬레이션용 단순 맵 생성기입니다.
  초록 주차 슬롯, 파란 대기공간도 색상이 강하면 장애물로 잡힐 수 있습니다.
  이후에는 색상별 분리 또는 YOLO/segmentation으로 교체하면 됩니다.
"""

import math
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import OccupancyGrid, Odometry


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

        # Topic names
        self.declare_parameter('image_topic', '/cctv/image')
        self.declare_parameter('front_odom_topic', '/front/odom')
        self.declare_parameter('rear_odom_topic', '/rear/odom')
        self.declare_parameter('map_topic', '/parking/map')

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

        self.image_topic = str(gp('image_topic').value)
        self.front_odom_topic = str(gp('front_odom_topic').value)
        self.rear_odom_topic = str(gp('rear_odom_topic').value)
        self.map_topic = str(gp('map_topic').value)

        self.front_pos = None
        self.rear_pos = None

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

        self.frame_n = 0

        self.get_logger().info(
            'cctv_map_builder started | '
            f'image={self.image_topic}, map={self.map_topic}, '
            f'cam=({self.cam_x:.2f},{self.cam_y:.2f},{self.cam_z:.2f}), '
            f'map={self.map_w_m:.2f}x{self.map_h_m:.2f}m, res={self.res:.3f}'
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
        # 1) 장애물 후보 분할: 채도 비슷한 지표 max(RGB)-min(RGB)
        # =====================================================
        imax = img.max(axis=2).astype(np.int16)
        imin = img.min(axis=2).astype(np.int16)
        obstacle = (imax - imin) > self.sat_th

        # =====================================================
        # 2) 픽셀 좌표 → 월드 좌표
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
        # 3) OccupancyGrid 채우기
        # =====================================================
        grid = np.zeros((self.gh, self.gw), dtype=np.int8)

        gx = ((wx - self.ox) / self.res).astype(np.int32)
        gy = ((wy - self.oy) / self.res).astype(np.int32)

        ok = (gx >= 0) & (gx < self.gw) & (gy >= 0) & (gy < self.gh)
        grid[gy[ok], gx[ok]] = 100

        # =====================================================
        # 4) 로봇 자신 마스킹
        # =====================================================
        for pos in (self.front_pos, self.rear_pos):
            if pos is None:
                continue

            r = int(round(self.mask_r / self.res))
            cx = int(round((pos[0] - self.ox) / self.res))
            cy = int(round((pos[1] - self.oy) / self.res))

            y0 = max(0, cy - r)
            y1 = min(self.gh, cy + r + 1)
            x0 = max(0, cx - r)
            x1 = min(self.gw, cx + r + 1)

            grid[y0:y1, x0:x1] = 0

        # =====================================================
        # 5) OccupancyGrid 발행
        # =====================================================
        m = OccupancyGrid()
        m.header.stamp = self.get_clock().now().to_msg()
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
        self.get_logger().info(
            f'published {self.map_topic} | occupied_cells={occ}',
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
