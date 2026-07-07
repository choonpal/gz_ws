# parking_gz_sim — 제어부 가제보 검증 패키지

협동 주차로봇(명세 v1.5)의 **제어 파트만** 떼어내 Gazebo Harmonic에서 증명하는 패키지.
인지부(YOLO/CCTV/ArUco)는 전부 생략하고, `waypoint_publisher`(가짜 관제)가 A* 경로를 발행하면
`sim_rigid_body_sync`가 Pure Pursuit + 강체 분배 + 거리/yaw 동기 보정으로 두 로봇을 몰고 간다.

## 환경

Ubuntu 24.04 + ROS2 Jazzy + **Gazebo Harmonic** (Classic 아님!)

```bash
sudo apt install ros-jazzy-ros-gz
```

## 빌드 & 실행

```bash
mkdir -p ~/gz_ws/src && cp -r parking_gz_sim ~/gz_ws/src/
cd ~/gz_ws && colcon build --packages-select parking_gz_sim
source install/setup.bash
ros2 launch parking_gz_sim sim.launch.py
```

가제보 창에서: 주황 상자 = Front, 파랑 상자 = Rear. 두 로봇이 대기공간(파란 패드)에서
출발해 기둥을 피해 slot_2(초록 패드) 앞 (2.5, 3.0)까지 0.25m 간격을 유지하며 이동하면 성공.

## 검증 방법 (합격 기준)

```bash
# 동기 오차 모니터링 — 명세 한계: 거리오차 30mm
ros2 topic echo /sync/error_state

# 도착 신호
ros2 topic echo /sync/goal_reached

# 비상정지 테스트 (즉시 정지해야 함)
ros2 topic pub --once /emergency_stop std_msgs/msg/Bool "data: true"
```

| 검증 항목 | 합격 기준 |
|-----------|-----------|
| 목표 도착 | 최종 오차 ≤ 3cm |
| 거리 동기 | 주행 중 dist_err ≤ 30mm (명세 부록 한계) |
| yaw 동기 | 주행 중 yaw_err ≤ 5° |
| 기둥 회피 | 기둥과 최소 이격 유지 (경로가 기둥을 우회) |
| E-Stop | 발행 즉시 두 로봇 정지 |

## 검증 사다리 (순서대로)

1. **로봇 1대 응답**: sync 노드 끄고 `ros2 topic pub /front/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}"` → 이동 + `/front/odom` 값 변화 확인
2. **직진 동기**: waypoint_publisher 파라미터를 직선 목표(goal 5.0, 0.6)로 바꿔 실행
3. **회전 경로**: 기본 A* 경로(대각+회전 포함)로 강체 분배(±ωL/2) 동작 확인
4. **전체 추종**: 기본 설정 그대로 — 기둥 회피 + 도착 정밀도
5. **(확장) 차량 운반**: 아래 DetachableJoint 참고

## 실기체와의 대응 관계

| 시뮬 구성 | 실기체 대응 |
|-----------|------------|
| VelocityControl 플러그인 | STM32 역기구학 + 바퀴 PID |
| OdometryPublisher | stm32_bridge + encoder_odometry |
| odom 초기위치 오프셋 파라미터 | CCTV 절대 보정 (vehicle_pose_feedback) |
| odom 간 직접 거리 계산 | Encoder + ArUco 칼만 융합 |
| waypoint_publisher | fleet_manager_node |

즉 이 시뮬에서 통과한 `sim_rigid_body_sync`의 제어 골격(Pure Pursuit → 강체 분배 →
동기 보정)은 실전 `rigid_body_sync_node`에 그대로 이식하고, 표의 왼쪽 요소들만
오른쪽 실물로 갈아 끼우면 된다.

## 2단계 확장
**cctv로 맵만들기** : cctv로 bev를 통해서, 맵을 만든뒤,주행
**바퀴 물리 포함 (MecanumDrive)**: VelocityControl은 바퀴를 생략하고 모델에 속도를
직접 인가한다(제어 로직 검증엔 이게 정확함). 바퀴 미끄러짐까지 보고 싶으면 로봇 모델에
바퀴 4개 링크 + 조인트를 추가하고 플러그인을 `gz::sim::systems::MecanumDrive`로 교체.

**차량 운반 (DetachableJoint)**: 리프팅 물리(마찰로 들기)는 시뮬에서 비현실적.
대신 로봇 모델에 아래를 넣고, "그립" 시점은 조인트가 붙은 상태로 시작 → 목표 도착 후
detach 토픽 발행으로 하차를 재현:

```xml
<plugin filename="gz-sim-detachable-joint-system" name="gz::sim::systems::DetachableJoint">
  <parent_link>base_link</parent_link>
  <child_model>target_vehicle</child_model>
  <child_link>body</child_link>
  <detach_topic>/front_robot/detach</detach_topic>
</plugin>
```
(차량 모델은 static=false로 바꾸고 로봇 사이 위치로 이동)

**노이즈 주입**: 지금 odom은 완벽한 ground truth다. 실기체 조건에 가깝게 하려면
sim_rigid_body_sync의 odom 콜백에 가우시안 노이즈/드리프트를 추가해 칼만 융합의
필요성까지 재현할 수 있다.

## 트러블슈팅

- `gz: command not found` → Harmonic 미설치. `sudo apt install gz-harmonic` 또는 ros-jazzy-ros-gz가 의존으로 설치함
- 로봇이 안 움직임 → `ros2 topic echo /front/odom`으로 브리지 확인. 안 나오면 bridge.yaml 경로/토픽명 점검
- 로봇이 엉뚱한 데로 감 → 월드 SDF의 로봇 초기 pose와 sync 노드의 `*_init_*` 파라미터 불일치
- 시간이 안 흐름 → 가제보 하단 재생(▶) 버튼, launch의 `-r` 플래그 확인
- PID가 이상하게 진동 → `use_sim_time`이 모든 노드에 True인지 확인 (dt 계산 꼬임)

---

# deliverybot 맵 버전 — CCTV로 맵 생성 → 협조 이동

`foiegreis/ros2_deliverybot_ws`의 ParkingLot.world 레이아웃(차량 4대 + 콘 5개 좌표)을
추출해 Harmonic용으로 재구성한 버전. **원본 레포를 직접 쓰지 않는 이유:**
바닥 메시(ParkingLot.dae)가 작성자 PC 절대경로라 레포에 없고, Gazebo Classic
포맷이라 Jazzy+Harmonic에서 재질/모델 경로가 깨진다. 좌표만 살려 재제작했다.

## 실행

```bash
ros2 launch parking_gz_sim deliverybot.launch.py
```

파이프라인: CCTV(상공 55m 수직 카메라, 5Hz) → `cctv_map_builder`(채도 분할 →
OccupancyGrid `/parking/map`) → `map_astar_planner`(팽창 2m + A*) →
`sim_rigid_body_sync`(휠베이스 2.5m, 최대 1m/s).

기본 목표는 (20, 20). **원하는 좌표로 보내기:**

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: 25.0, y: 30.0}}}"
```

또는 RViz2에서 `/parking/map`을 Map으로 띄우고 'Goal Pose' 클릭 (같은 토픽).

## 확인 포인트

```bash
ros2 topic echo /parking/map --field info      # 맵 생성 확인 (320x500, 0.1m)
ros2 run rqt_image_view rqt_image_view /cctv/image_raw   # CCTV 화면
ros2 topic echo /sync/error_state              # 동기 오차
```

- 흰 주차선이 맵에 장애물로 안 잡히는지 (채도 분할 자체 검증)
- 로봇 2대 자신이 맵에서 지워지는지 (마스킹 = 실전의 odom-CCTV 매칭 문제)
- 새 goal_pose 발행 시 재계획되는지

## 실전 대응 관계 (추가)

| 시뮬 | 실전 명세 |
|------|----------|
| 채도 임계 분할 | YOLO11n-seg 검출 |
| 수직카메라 픽셀 스케일 변환 | Homography (calibrate.py) |
| 로봇 마스킹 (odom 기반) | odom-CCTV 매칭 |
| /goal_pose 수동 발행 | 빈자리 자동 선정 |

## 오프라인 검증 결과 (합성 이미지 폐루프)

- 픽셀 스케일 4.99cm/px, 커버리지 49.9×31.9m
- 주차선 무시 / 차량 검출 / 로봇 마스킹 통과
- A* waypoint 27개, 경로-차량 최소거리 4.73m
- (2,-5) → (20,20) 34.4초 도착, 오차 14.9cm, 주행 중 거리오차 최대 2.9mm
