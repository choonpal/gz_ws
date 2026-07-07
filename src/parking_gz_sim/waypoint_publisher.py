#!/usr/bin/env python3
"""
waypoint_publisher.py — 가짜 fleet_manager (시뮬레이션 검증용)

인지부(YOLO/CCTV) 없이 명세 7-2의 핵심만 수행:
  하드코딩된 맵(기둥 위치) 위에서 A* 경로 계산
  → /virtual_robot/waypoints (nav_msgs/Path) 발행

목표 슬롯도 파라미터로 하드코딩 (기본: slot_2 앞 (2.5, 3.0)).
실전에서는 이 노드가 진짜 fleet_manager_node로 교체된다.
"""

import math
import heapq

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


class WaypointPublisher(Node):
    def __init__(self):
        super().__init__('waypoint_publisher')

        # ===== 파라미터 (월드 SDF와 일치해야 함) =====
        self.declare_parameter('start_x', 0.625)   # 가상 중심 초기위치
        self.declare_parameter('start_y', 0.60)
        self.declare_parameter('goal_x', 2.5)      # slot_2 앞
        self.declare_parameter('goal_y', 3.0)
        self.declare_parameter('map_w', 6.0)
        self.declare_parameter('map_h', 4.0)
        self.declare_parameter('resolution', 0.1)
        # 기둥 [x, y, r] * N (평탄화)
        self.declare_parameter('pillars', [3.0, 2.0, 0.08, 4.2, 1.2, 0.08])
        self.declare_parameter('inflation', 0.35)  # 로봇쌍 반폭 + 여유

        self.res = self.get_parameter('resolution').value
        self.map_w = self.get_parameter('map_w').value
        self.map_h = self.get_parameter('map_h').value
        self.gw = int(self.map_w / self.res)
        self.gh = int(self.map_h / self.res)

        self.grid = self._build_grid()
        self.path_msg = self._plan()

        self.pub = self.create_publisher(Path, '/virtual_robot/waypoints', 10)
        # QoS 문제를 피하려고 1Hz로 계속 재발행 (구독자가 늦게 떠도 받도록)
        self.create_timer(1.0, self._publish)
        self.get_logger().info(
            f'waypoint_publisher 시작 — waypoint {len(self.path_msg.poses)}개')

    # ---------- 격자 생성 ----------
    def _build_grid(self):
        grid = [[0] * self.gw for _ in range(self.gh)]
        pillars = self.get_parameter('pillars').value
        infl = self.get_parameter('inflation').value
        for i in range(0, len(pillars), 3):
            px, py, pr = pillars[i], pillars[i + 1], pillars[i + 2]
            r = pr + infl
            for gy in range(self.gh):
                for gx in range(self.gw):
                    wx = (gx + 0.5) * self.res
                    wy = (gy + 0.5) * self.res
                    if math.hypot(wx - px, wy - py) < r:
                        grid[gy][gx] = 1
        # 외곽 여유 (벽에서 0.2m)
        margin = int(0.2 / self.res)
        for gy in range(self.gh):
            for gx in range(self.gw):
                if gx < margin or gx >= self.gw - margin or \
                   gy < margin or gy >= self.gh - margin:
                    grid[gy][gx] = 1
        return grid

    # ---------- A* ----------
    def _astar(self, start, goal):
        def h(a, b):
            return math.hypot(a[0] - b[0], a[1] - b[1])
        nbrs = [(-1, 0, 1), (1, 0, 1), (0, -1, 1), (0, 1, 1),
                (-1, -1, 1.414), (-1, 1, 1.414), (1, -1, 1.414), (1, 1, 1.414)]
        openq = [(h(start, goal), 0.0, start, None)]
        came, gscore = {}, {start: 0.0}
        while openq:
            _, g, cur, parent = heapq.heappop(openq)
            if cur in came:
                continue
            came[cur] = parent
            if cur == goal:
                path = []
                while cur is not None:
                    path.append(cur)
                    cur = came[cur]
                return path[::-1]
            for dx, dy, c in nbrs:
                nx, ny = cur[0] + dx, cur[1] + dy
                if not (0 <= nx < self.gw and 0 <= ny < self.gh):
                    continue
                if self.grid[ny][nx]:
                    continue
                ng = g + c
                if ng < gscore.get((nx, ny), 1e18):
                    gscore[(nx, ny)] = ng
                    heapq.heappush(openq, (ng + h((nx, ny), goal), ng,
                                           (nx, ny), cur))
        return None

    def _w2g(self, x, y):
        return (min(self.gw - 1, max(0, int(x / self.res))),
                min(self.gh - 1, max(0, int(y / self.res))))

    def _plan(self):
        sx = self.get_parameter('start_x').value
        sy = self.get_parameter('start_y').value
        gx = self.get_parameter('goal_x').value
        gy = self.get_parameter('goal_y').value

        cells = self._astar(self._w2g(sx, sy), self._w2g(gx, gy))
        msg = Path()
        msg.header.frame_id = 'world'
        if cells is None:
            self.get_logger().error('A* 경로 실패! 기둥/여유 설정 확인')
            return msg
        # 3셀 간격으로 다운샘플 + 마지막에 정확한 목표점
        for i in range(0, len(cells), 3):
            cx, cy = cells[i]
            p = PoseStamped()
            p.header.frame_id = 'world'
            p.pose.position.x = (cx + 0.5) * self.res
            p.pose.position.y = (cy + 0.5) * self.res
            p.pose.orientation.w = 1.0
            msg.poses.append(p)
        p = PoseStamped()
        p.header.frame_id = 'world'
        p.pose.position.x = gx
        p.pose.position.y = gy
        p.pose.orientation.w = 1.0
        msg.poses.append(p)
        return msg

    def _publish(self):
        self.path_msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.path_msg)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
