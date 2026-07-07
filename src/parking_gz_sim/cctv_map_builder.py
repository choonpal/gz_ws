#!/usr/bin/env python3
"""
cctv_map_builder.py — CCTV 이미지 → OccupancyGrid (명세 7-1 yolo_bev_map의 시뮬 대역)

파이프라인 (실전과 1:1 대응):
  카메라 이미지 수신
  → 장애물 분할 (시뮬: 채도 임계값 / 실전: YOLO11n-seg)
  → 픽셀 → 월드좌표 변환 (시뮬: 수직카메라 핀홀 스케일 / 실전: Homography)
  → 로봇 자신 마스킹 (odom-CCTV 매칭 — 실전과 동일한 문제!)
  → OccupancyGrid /parking/map 발행

채도 기준을 쓰는 이유:
  바닥(어두운 회색)과 주차선(흰색)은 무채색 → 무시
  차량·콘·로봇은 유채색 → 장애물
  = 흰 주차선이 장애물로 오검출되지 않는지가 이 노드의 자체 검증 포인트
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

        # ===== 카메라 파라미터 (월드 SDF와 일치) =====
        self.declare_parameter('cam_x', 14.0)
        self.declare_parameter('cam_y', 12.0)
        self.declare_parameter('cam_z', 55.0)
        self.declare_parameter('hfov', 0.852)
        # ===== 맵 파라미터 =====
        self.declare_parameter('origin_x', -2.0)
        self.declare_parameter('origin_y', -13.0)
        self.declare_parameter('map_w_m', 32.0)
        self.declare_parameter('map_h_m', 50.0)
        self.declare_parameter('resolution', 0.1)
        # ===== 분할/마스킹 =====
        self.declare_parameter('sat_threshold', 40)    # 채도(max-min) 임계
        self.declare_parameter('robot_mask_radius', 1.2)
        # 로봇 초기위치 (odom → 전역 복원, 월드 SDF와 일치)
        self.declare_parameter('front_init_x', 3.25)
        self.declare_parameter('front_init_y', -5.0)
        self.declare_parameter('rear_init_x', 0.75)
        self.declare_parameter('rear_init_y', -5.0)

        gp = self.get_parameter
        self.cam = (gp('cam_x').value, gp('cam_y').value, gp('cam_z').value)
        self.hfov = gp('hfov').value
        self.ox = gp('origin_x').value
        self.oy = gp('origin_y').value
        self.res = gp('resolution').value
        self.gw = int(gp('map_w_m').value / self.res)
        self.gh = int(gp('map_h_m').value / self.res)
        self.sat_th = gp('sat_threshold').value
        self.mask_r = gp('robot_mask_radius').value

        self.front = None
        self.rear = None

        self.create_subscription(Image, '/cctv/image_raw', self.image_cb, 1)
        self.create_subscription(Odometry, '/front/odom',
                                 lambda m: self.odom_cb('front', m), 10)
        self.create_subscription(Odometry, '/rear/odom',
                                 lambda m: self.odom_cb('rear', m), 10)
        self.pub_map = self.create_publisher(OccupancyGrid, '/parking/map', 1)

        self.frame_n = 0
        self.get_logger().info('cctv_map_builder 시작 — /cctv/image_raw 대기')

    def odom_cb(self, role, msg):
        p = msg.pose.pose.position
        gp = self.get_parameter
        if role == 'front':
            self.front = (gp('front_init_x').value + p.x,
                          gp('front_init_y').value + p.y)
        else:
            self.rear = (gp('rear_init_x').value + p.x,
                         gp('rear_init_y').value + p.y)

    # =====================================================
    def image_cb(self, msg):
        self.frame_n += 1
        if self.frame_n % 2:      # 5Hz 입력 → 약 2.5Hz 처리
            return

        H, W = msg.height, msg.width
        img = np.frombuffer(msg.data, dtype=np.uint8)
        if msg.encoding not in ('rgb8', 'bgr8'):
            self.get_logger().warn(f'미지원 인코딩 {msg.encoding}',
                                   throttle_duration_sec=5.0)
            return
        step = msg.step if msg.step else W * 3
        img = img.reshape(H, step)[:, :W * 3].reshape(H, W, 3)

        # ---- 1) 장애물 분할: 채도(max-min) 임계 ----
        imax = img.max(axis=2).astype(np.int16)
        imin = img.min(axis=2).astype(np.int16)
        obstacle = (imax - imin) > self.sat_th   # 유채색 = 장애물

        # ---- 2) 픽셀 → 월드좌표 (수직 카메라 핀홀) ----
        # 카메라 pose (0, pi/2, 0): u축 → 월드 -y, v축 → 월드 -x
        s = 2.0 * self.cam[2] * math.tan(self.hfov / 2.0) / W  # m/px
        vs, us = np.nonzero(obstacle)
        wx = self.cam[0] - (vs - H / 2.0 + 0.5) * s
        wy = self.cam[1] - (us - W / 2.0 + 0.5) * s

        # ---- 3) 격자 채우기 ----
        grid = np.zeros((self.gh, self.gw), dtype=np.int8)
        gx = ((wx - self.ox) / self.res).astype(np.int32)
        gy = ((wy - self.oy) / self.res).astype(np.int32)
        ok = (gx >= 0) & (gx < self.gw) & (gy >= 0) & (gy < self.gh)
        grid[gy[ok], gx[ok]] = 100

        # ---- 4) 로봇 자신 마스킹 (실전의 odom-CCTV 매칭에 해당) ----
        for pos in (self.front, self.rear):
            if pos is None:
                continue
            r = int(self.mask_r / self.res)
            cx = int((pos[0] - self.ox) / self.res)
            cy = int((pos[1] - self.oy) / self.res)
            y0, y1 = max(0, cy - r), min(self.gh, cy + r + 1)
            x0, x1 = max(0, cx - r), min(self.gw, cx + r + 1)
            grid[y0:y1, x0:x1] = 0

        # ---- 5) 발행 ----
        m = OccupancyGrid()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'map'
        m.info.resolution = self.res
        m.info.width = self.gw
        m.info.height = self.gh
        m.info.origin.position.x = self.ox
        m.info.origin.position.y = self.oy
        m.info.origin.orientation.w = 1.0
        m.data = grid.flatten().tolist()
        self.pub_map.publish(m)

        occ = int((grid > 0).sum())
        self.get_logger().info(f'맵 발행 — 점유셀 {occ}개',
                               throttle_duration_sec=5.0)


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
