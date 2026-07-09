# 차량 인지 파트 패치 — 대기공간 정차 감지 → 리프트 허가

교체/추가 3곳: `cctv_map_builder.py`(전체 교체본 별첨), `parking_lot.sdf`(모델 추가),
`map_astar_planner.py`(게이트 추가).

---

## 1. parking_lot.sdf — target_vehicle 모델 추가

`</world>` 닫는 태그 직전에 삽입:

```xml
    <!-- ===== Target Vehicle (대기공간 정차 차량) ===== -->
    <!-- 실제 시스템처럼 로봇이 차량 밑으로 들어가 바퀴를 잡는 구조를
         재현하기 위해 차체를 로봇 높이 위에 static으로 띄워둔다.
         z=0.30은 로봇 박스 높이보다 높게 — 로봇 모델 높이에 맞게 조정할 것.
         나중에 운반 단계에서 DetachableJoint의 child_model로 그대로 사용. -->
    <model name="target_vehicle">
      <static>true</static>
      <pose>0.625 0.6 0.30 0 0 0</pose>
      <link name="body">
        <visual name="body_visual">
          <geometry>
            <box><size>0.45 0.22 0.08</size></box>
          </geometry>
          <material>
            <ambient>0.8 0.05 0.05 1</ambient>
            <diffuse>0.9 0.05 0.05 1</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
          </material>
        </visual>
        <collision name="body_collision">
          <geometry>
            <box><size>0.45 0.22 0.08</size></box>
          </geometry>
        </collision>
      </link>
    </model>
```

주의:
- **빨간색(0.9, 0.05, 0.05)이 차량 클래스 판별 기준.** 다른 색으로 바꾸면
  `veh_r_min/veh_g_max/veh_b_max` 파라미터도 같이 조정해야 한다.
- 위치 (0.625, 0.6) = Front(0.75)와 Rear(0.50) 사이 중앙, 대기공간 위.
- 로봇 초기 위치가 이미 그립 자세이므로 차량이 로봇을 CCTV에서 가리는데,
  로봇 마스킹은 odom 기반이라 영향 없음 (실전과 동일한 상황).

## 2. map_astar_planner.py — 리프트 허가 게이트

목적: target_ready(= 실전의 /robot/lifted 게이트 대응) 전에는 경로계획 금지.

(1) import 추가:
```python
from std_msgs.msg import Bool
```

(2) `__init__` 파라미터/구독 추가 (기존 구독들 옆에):
```python
        self.declare_parameter('require_target', True)
        self.target_ready = False
        self.create_subscription(Bool, '/parking/target_ready',
                                 self.ready_cb, 10)
```

(3) 콜백 추가:
```python
    def ready_cb(self, msg):
        if msg.data and not self.target_ready:
            self.target_ready = True
            self.get_logger().info('target_ready 수신 — 경로계획 허가')
```

(4) `tick()` 맨 앞에 게이트 삽입:
```python
        if self.get_parameter('require_target').value and not self.target_ready:
            return
```

기존 단독 테스트(차량 없이 goal만 찍기)를 유지하고 싶으면 launch에서
`require_target: False`.

---

## 3. 테스트 절차

```bash
colcon build --packages-select parking_gz_sim && source install/setup.bash
ros2 launch parking_gz_sim <해당 launch>
```

### 합격 기준

| 항목 | 확인 방법 | 기대 결과 |
|------|-----------|-----------|
| 차량 검출 | `ros2 topic echo /parking/target_ready` | 시작 ~2초 후 `data: true` (stop_time 경과) |
| 위치 정확도 | `ros2 topic echo /parking/target_pose --once` | (0.625, 0.6) ± 5cm |
| 게이트 동작 | planner 로그 | ready 전엔 "경로 계획 완료" 로그 없음 → ready 후 계획 시작 |
| 차량 ≠ 장애물 | RViz2 `/parking/map` | 대기공간 차량 자리가 비어 있음 |
| 정차 판정 (이동 중 거부) | 아래 명령으로 차량 이동 후 관찰 | 이동 직후 ready 판정 이력 리셋, 2초 재정지 후에만 확정 |

### 이동 차량 테스트 (정차 판정 검증)

ready가 latch되기 전에 (재시작 직후) 차량을 움직여본다:

```bash
gz service -s /world/parking_lot/set_pose \
  --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 300 \
  --req 'name: "target_vehicle", position: {x: 1.0, y: 0.6, z: 0.30}'
```

이동 프레임마다 stationarity 이력이 리셋되므로, 멈춘 뒤 stop_time(2초)이
지나야만 TARGET_READY가 뜬다. ROI(waiting_zone) 밖으로 옮기면 아예 판정 안 함.

### 파라미터 요약 (cctv_map_builder 신규)

| 파라미터 | 기본값 | 의미 |
|----------|--------|------|
| waiting_zone | [0.0, 0.0, 1.3, 1.2] | 대기공간 ROI [x1,y1,x2,y2] |
| veh_r_min / veh_g_max / veh_b_max | 120 / 80 / 80 | 빨간 차량 픽셀 임계 |
| min_vehicle_px | 30 | 노이즈 배제 최소 픽셀 |
| stop_eps | 0.02 | 정차 판정 이동 허용 [m] |
| stop_time | 2.0 | 정차 판정 지속 시간 [s] |
| vehicle_mask_radius | 0.30 | 맵에서 target 차량 마스킹 반경 [m] |

---

## 4. 실전 대응 관계 (README 표에 추가할 행)

| 시뮬 | 실전 명세 |
|------|----------|
| 빨간색 픽셀 분류 | YOLO11n-seg vehicle 클래스 |
| 정차 판정 (2cm/2s 히스테리시스) | 동일 로직 (YOLO bbox 지터 흡수) |
| /parking/target_ready | fleet WAIT_TARGET → 리프트 허가 상태 전이 |
| ready 후 target_pose 지속 발행 | /parking/vehicle_pose_feedback |
| target 차량 맵 마스킹 | 운반 차량 비장애물 처리 |
