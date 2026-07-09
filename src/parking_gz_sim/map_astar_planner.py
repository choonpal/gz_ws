#!/usr/bin/env python3
"""
map_astar_planner.py — CCTV 맵 기반 A* 경로계획 (명세 7-2 fleet_manager의 시뮬 대역)

waypoint_publisher(기둥 하드코딩)와 달리:
  /parking/map (OccupancyGrid, CCTV가 만든 맵) 구독
  /goal_pose (PoseStamped, RViz2 'Goal Pose' 클릭과 호환) 구독
  → 팽창(inflation) → 격자 다운샘플 → A* → /virtual_robot/waypoints 발행

2단계 상태 흐름 (require_target=True):
  대기 → /parking/target_ready 수신
  → [to_target] 차량 +x쪽 축선상 정렬점(standoff)으로 경로계획.
    차량 중심을 직접 목표로 하면 로봇이 옆에서 차체/바퀴에 부딪혀
    차량을 밀게 되므로, 반드시 차량 밖에서 멈춘다.
    이후 차량 밑 삽입은 gripper_controller가 저속 직진으로 수행.
  → gripper_controller가 삽입·정렬·파지·결합 후 /robot/lifted 발행
  → [to_goal] 최종 목표(/goal_pose 또는 default_goal)로 운반 경로계획

로봇 현재 위치는 두 로봇 odom의 중점(가상 강체 중심)으로 계산.
"""

import math
import heapq
import numpy as np

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path, Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool


class MapAstarPlanner(Node):
    def __init__(self):
        super().__init__('map_astar_planner')

        self.declare_parameter('inflation', 2.0)     # 로봇쌍 반폭 + 여유 (m)
        self.declare_parameter('plan_res', 0.5)      # 계획용 격자 (m)
        self.declare_parameter('default_goal_x', 20.0)
        self.declare_parameter('default_goal_y', 20.0)
        # 차량 접근 정렬점: 차량 중심에서 +x로 띄우는 거리 (m)
        # gripper_controller의 approach_standoff와 반드시 같아야 한다.
        self.declare_parameter('approach_standoff', 0.60)
        self.map_msg = None
        self.front = None
        self.rear = None
        self.goal = None          # 현재 추종 중인 목표
        self.user_goal = None     # /goal_pose로 받은 최종 목표
        self.target_xy = None     # /parking/target_pose (target 차량 위치)
        self.path_msg = None
        self.planned_goal = None

        # 리프트 허가 게이트 (실전의 /robot/lifted 게이트 대응)
        self.declare_parameter('require_target', True)
        self.target_ready = False
        self.create_subscription(Bool, '/parking/target_ready',
                                 self.ready_cb, 10)

        # 2단계 상태:
        #   to_target — target_ready 후 대기공간 차량 밑(그립 위치)으로 이동
        #   to_goal   — 그립·리프트 완료(/robot/lifted) 후 최종 목표로 운반
        # require_target=False(단독 테스트)면 바로 to_goal에서 시작.
        self.phase = ('to_target'
                      if self.get_parameter('require_target').value
                      else 'to_goal')
        self.create_subscription(PoseStamped, '/parking/target_pose',
                                 self.target_pose_cb, 10)
        self.create_subscription(Bool, '/robot/lifted',
                                 self.lifted_cb, 10)

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
    def ready_cb(self, msg):
        if msg.data and not self.target_ready:
            self.target_ready = True
            self.get_logger().info('target_ready 수신 — 경로계획 허가')

    def map_cb(self, msg):
        self.map_msg = msg

    def target_pose_cb(self, msg):
        self.target_xy = (msg.pose.position.x, msg.pose.position.y)

    def lifted_cb(self, msg):
        """그립·리프트 완료(/robot/lifted) → 최종 목표로 운반 시작."""
        if msg.data and self.phase == 'to_target':
            self.phase = 'to_goal'
            self.get_logger().info('리프트 완료 수신 — 최종 목표로 운반 시작')

    def goal_cb(self, msg):
        self.user_goal = (msg.pose.position.x, msg.pose.position.y)
        self.get_logger().info(
            f'새 목표: ({self.user_goal[0]:.1f}, {self.user_goal[1]:.1f})')

    def odom_cb(self, role, msg):
        # /front/odom, /rear/odom은 이미 world/map 좌표로 들어온다
        # (cctv_map_builder와 동일 가정). init 값을 더하면 두 번 더해져
        # 시작 위치가 틀어진다.
        p = msg.pose.pose.position
        if role == 'front':
            self.front = (float(p.x), float(p.y))
        else:
            self.rear = (float(p.x), float(p.y))

    # ---------------- 주기 처리 ----------------
    def tick(self):
        if self.get_parameter('require_target').value and not self.target_ready:
            return
        if self.map_msg is None or self.front is None or self.rear is None:
            return

        if self.phase == 'to_target':
            # 1단계: 차량 +x쪽 정렬점(standoff)까지만 — 차량 밑 삽입은
            # gripper_controller 담당 (차량 중심을 목표로 하면 밀어버림)
            if self.target_xy is None:
                return
            so = self.get_parameter('approach_standoff').value
            self.goal = (self.target_xy[0] + so, self.target_xy[1])
        else:
            # 2단계: 최종 목표 (RViz goal_pose 우선, 없으면 기본값)
            gp = self.get_parameter
            self.goal = self.user_goal or (gp('default_goal_x').value,
                                           gp('default_goal_y').value)

        # target_pose의 픽셀 지터로 매 tick 재계획하지 않게 5cm 이상 바뀔 때만
        if self.planned_goal is None or \
                math.hypot(self.planned_goal[0] - self.goal[0],
                           self.planned_goal[1] - self.goal[1]) > 0.05:
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
