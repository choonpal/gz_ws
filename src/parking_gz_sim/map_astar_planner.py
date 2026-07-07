#!/usr/bin/env python3
"""
map_astar_planner.py — CCTV 맵 기반 A* 경로계획 (명세 7-2 fleet_manager의 시뮬 대역)

waypoint_publisher(기둥 하드코딩)와 달리:
  /parking/map (OccupancyGrid, CCTV가 만든 맵) 구독
  /goal_pose (PoseStamped, RViz2 'Goal Pose' 클릭과 호환) 구독
  → 팽창(inflation) → 0.5m 격자로 다운샘플 → A* → /virtual_robot/waypoints 발행

로봇 현재 위치는 두 로봇 odom의 중점(가상 강체 중심)으로 계산.
"""

import math
import heapq
import numpy as np

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path, Odometry
from geometry_msgs.msg import PoseStamped


class MapAstarPlanner(Node):
    def __init__(self):
        super().__init__('map_astar_planner')

        self.declare_parameter('inflation', 2.0)     # 로봇쌍 반폭 + 여유 (m)
        self.declare_parameter('plan_res', 0.5)      # 계획용 격자 (m)
        self.declare_parameter('default_goal_x', 20.0)
        self.declare_parameter('default_goal_y', 20.0)
        self.declare_parameter('front_init_x', 3.25)
        self.declare_parameter('front_init_y', -5.0)
        self.declare_parameter('rear_init_x', 0.75)
        self.declare_parameter('rear_init_y', -5.0)

        self.map_msg = None
        self.front = None
        self.rear = None
        self.goal = None
        self.path_msg = None
        self.planned_goal = None

        self.create_subscription(OccupancyGrid, '/parking/map',
                                 self.map_cb, 1)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_cb, 10)
        self.create_subscription(Odometry, '/front/odom',
                                 lambda m: self.odom_cb('front', m), 10)
        self.create_subscription(Odometry, '/rear/odom',
                                 lambda m: self.odom_cb('rear', m), 10)
        self.pub_path = self.create_publisher(Path, '/virtual_robot/waypoints', 10)

        self.create_timer(1.0, self.tick)
        self.get_logger().info('map_astar_planner 시작 — CCTV 맵 대기')

    # ---------------- 콜백 ----------------
    def map_cb(self, msg):
        self.map_msg = msg

    def goal_cb(self, msg):
        self.goal = (msg.pose.position.x, msg.pose.position.y)
        self.planned_goal = None   # 재계획 트리거
        self.get_logger().info(f'새 목표: ({self.goal[0]:.1f}, {self.goal[1]:.1f})')

    def odom_cb(self, role, msg):
        p = msg.pose.pose.position
        gp = self.get_parameter
        if role == 'front':
            self.front = (gp('front_init_x').value + p.x,
                          gp('front_init_y').value + p.y)
        else:
            self.rear = (gp('rear_init_x').value + p.x,
                         gp('rear_init_y').value + p.y)

    # ---------------- 주기 처리 ----------------
    def tick(self):
        if self.goal is None:
            gp = self.get_parameter
            self.goal = (gp('default_goal_x').value, gp('default_goal_y').value)
        if self.map_msg is None or self.front is None or self.rear is None:
            return
        if self.planned_goal != self.goal:
            self.plan()
        if self.path_msg is not None:
            self.path_msg.header.stamp = self.get_clock().now().to_msg()
            self.pub_path.publish(self.path_msg)

    # ---------------- 계획 ----------------
    def plan(self):
        m = self.map_msg
        res = m.info.resolution
        ox, oy = m.info.origin.position.x, m.info.origin.position.y
        fine = np.array(m.data, dtype=np.int8).reshape(m.info.height,
                                                       m.info.width)
        occ = fine > 50

        # 0.5m 계획 격자로 다운샘플 (블록 내 하나라도 점유면 점유)
        pres = self.get_parameter('plan_res').value
        f = max(1, int(round(pres / res)))
        gh = m.info.height // f
        gw = m.info.width // f
        coarse = occ[:gh * f, :gw * f].reshape(gh, f, gw, f).max(axis=(1, 3))

        # 팽창: 점유셀 주변 inflation 반경 차단
        k = int(math.ceil(self.get_parameter('inflation').value / pres))
        infl = coarse.copy()
        base = coarse
        for dy in range(-k, k + 1):
            for dx in range(-k, k + 1):
                if dx == 0 and dy == 0:
                    continue
                sh = np.zeros_like(base)
                ys = slice(max(0, dy), gh + min(0, dy))
                yd = slice(max(0, -dy), gh + min(0, -dy))
                xs = slice(max(0, dx), gw + min(0, dx))
                xd = slice(max(0, -dx), gw + min(0, -dx))
                sh[yd, xd] = base[ys, xs]
                infl |= sh

        def w2g(x, y):
            return (min(gw - 1, max(0, int((x - ox) / pres))),
                    min(gh - 1, max(0, int((y - oy) / pres))))

        cx = (self.front[0] + self.rear[0]) / 2
        cy = (self.front[1] + self.rear[1]) / 2
        start = self.nearest_free(infl, w2g(cx, cy), gw, gh)
        goal_c = self.nearest_free(infl, w2g(*self.goal), gw, gh)
        if start is None or goal_c is None:
            self.get_logger().error('시작/목표 주변에 자유공간 없음')
            return

        cells = self.astar(infl, start, goal_c, gw, gh)
        if cells is None:
            self.get_logger().error('A* 경로 실패 — 목표가 막혀있을 수 있음')
            return

        path = Path()
        path.header.frame_id = 'map'
        for i in range(0, len(cells), 2):   # ~1m 간격 waypoint
            gx, gy = cells[i]
            p = PoseStamped()
            p.header.frame_id = 'map'
            p.pose.position.x = ox + (gx + 0.5) * pres
            p.pose.position.y = oy + (gy + 0.5) * pres
            p.pose.orientation.w = 1.0
            path.poses.append(p)
        p = PoseStamped()
        p.header.frame_id = 'map'
        p.pose.position.x, p.pose.position.y = self.goal
        p.pose.orientation.w = 1.0
        path.poses.append(p)

        self.path_msg = path
        self.planned_goal = self.goal
        self.get_logger().info(
            f'경로 계획 완료 — waypoint {len(path.poses)}개 '
            f'({cx:.1f},{cy:.1f}) → ({self.goal[0]:.1f},{self.goal[1]:.1f})')

    @staticmethod
    def nearest_free(grid, cell, gw, gh, max_r=10):
        x0, y0 = cell
        if not grid[y0, x0]:
            return cell
        for r in range(1, max_r + 1):
            for dy in range(-r, r + 1):
                for dx in range(-r, r + 1):
                    x, y = x0 + dx, y0 + dy
                    if 0 <= x < gw and 0 <= y < gh and not grid[y, x]:
                        return (x, y)
        return None

    @staticmethod
    def astar(grid, start, goal, gw, gh):
        h = lambda a, b: math.hypot(a[0] - b[0], a[1] - b[1])
        nb = [(-1, 0, 1), (1, 0, 1), (0, -1, 1), (0, 1, 1),
              (-1, -1, 1.414), (-1, 1, 1.414), (1, -1, 1.414), (1, 1, 1.414)]
        q = [(h(start, goal), 0.0, start, None)]
        came, gs = {}, {start: 0.0}
        while q:
            _, g, cur, par = heapq.heappop(q)
            if cur in came:
                continue
            came[cur] = par
            if cur == goal:
                p = []
                while cur is not None:
                    p.append(cur)
                    cur = came[cur]
                return p[::-1]
            for dx, dy, c in nb:
                nx, ny = cur[0] + dx, cur[1] + dy
                if not (0 <= nx < gw and 0 <= ny < gh) or grid[ny, nx]:
                    continue
                ng = g + c
                if ng < gs.get((nx, ny), 1e18):
                    gs[(nx, ny)] = ng
                    heapq.heappush(q, (ng + h((nx, ny), goal), ng,
                                       (nx, ny), cur))
        return None


def main(args=None):
    rclpy.init(args=args)
    node = MapAstarPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
