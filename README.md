# 제4회 국제 대학생 EV 자율주행 대회 1/5부문



## 개요

담당: 차량의 차선 주행 로직 + GPS 경로 주행 중 장애물 회피 로직


## 담당 범위

| 영역 | 패키지/모듈 | 구현한 내용 |
|---|---|---|
| Lane Perception | [`lane`](./lane) | 카메라 기반 차선 검출, BEV 변환, DBSCAN/RANSAC/Sliding Window/Kalman 기반 차선 포인트 추출, 차선 중심 경로 생성, Pure Pursuit 기반 차선 조향각 계산 |
| ROS 2 Interface | [`my_lane_msgs`](./my_lane_msgs) | 좌/우 차선 좌표 전달을 위한 `LanePoints.msg` 인터페이스 설계 및 패키지 구성 |
| GPS Path Localization | [`control/gps/mission_control.py`](./control/gps/mission_control.py) | CSV 기반 GPS waypoint를 로컬 XY로 변환하고, 실시간 GNSS 위치와 heading을 이용해 차량 중심 상대 좌표계로 재투영하여 `/gps_path`를 생성 |
| GPS Obstacle Avoidance | [`control/gps/mpc.py`](./control/gps/mpc.py) | LiDAR 장애물 포인트에 C-Space 기반 반경 확장을 적용하고, 1차원 각도 Cost Map을 생성한 뒤 MPC 기반 비용함수로 최적 회피 경로를 선택 |

## 주요 기능

Lane
- 전방 카메라 영상에서 좌/우 차선을 추출
- 차선 점들을 기반으로 중앙 주행 경로를 생성
- 생성된 경로에서 look-ahead target을 선택해 조향각을 계산
- 차선 검출 여부를 함께 발행해 lane 주행과 GPS 주행 전환에 활용

GPS + Obstacle Avoidance
- CSV 기반 GPS 절대 경로를 로컬 XY 좌표계로 변환
- GNSS 위치와 course heading을 이용해 waypoint를 차량 기준 상대 좌표로 변환
- 변환된 경로를 velodyne 프레임 기준 /gps_path로 발행
- LiDAR에서 검출된 상대 좌표 장애물 마커(`/cluster_markers`)를 받아 C-Space 기반으로 장애물 점유 각도를 확장
- 확장된 장애물 각도 분포를 1차원 Cost Map으로 만들고 Gaussian filter로 주변 위험도까지 반영
- 여러 조향각 시나리오를 예측하고 장애물 Cost, GPS 경로 이탈 Cost, 조향 변화 Cost를 합산해 최적 회피 경로를 선택
- 선택된 회피 경로를 Pure Pursuit로 추종해 최종 GPS+장애물 구간 조향 명령으로 연결



## 주요 기능

### 1. Lane 패키지 전체 구현

- 커스텀 YOLO 기반 차선 세그멘테이션 파이프라인 구현
- Bird's Eye View 변환으로 차선 geometry를 정규화
- DBSCAN, RANSAC, Kalman filter 기반으로 노이즈와 순간 검출 실패를 완화
- 좌/우 차선이 모두 있을 때는 중앙선 생성
- 한쪽 차선만 존재할 때는 offset 기반으로 가상 중심 경로 복원
- spline 기반 재샘플링으로 제어용 path 생성
- Pure Pursuit 기반 조향각 계산

### 2. 커스텀 메시지 인터페이스 구현

- 좌/우 차선 포인트를 각각 left_x, left_y, right_x, right_y 배열로 전달

### 3. GPS + 장애물 구간 자율주행 구현

- 이 기능의 흐름은 아래 3단계로 나뉩니다.

#### Step 1. 전역 경로의 로컬화 + 장애물 크기 확대

- GPS 기반 전역 waypoint를 로컬 XY 평면으로 변환한 뒤, 차량의 현재 GNSS 위치와 heading을 기준으로 차량 중심 상대 좌표계로 재투영
- `/gps_path`를 `velodyne` 기준으로 발행해 LiDAR 장애물과 동일 좌표계에서 경로를 다룰 수 있도록 구성
- LiDAR 클러스터 장애물 포인트마다 `base_radius`를 적용해 C-Space 개념으로 점유 각도를 확장

#### Step 2. Cost Map 생성

- 확장된 장애물이 차지하는 각도 구간을 기준으로 1차원 angular histogram 형태의 Cost Map 생성
- `gaussian_filter1d`를 적용해 특정 방향뿐 아니라 그 주변 방향의 위험도까지 함께 반영
- 생성된 Cost Map은 이후 예측 경로가 어느 방향으로 지나가는지 평가하는 장애물 Cost의 기준으로 사용

#### Step 3. 모델 예측 제어 기반 회피 경로 탐색

- 여러 가상의 조향각 후보를 샘플링하고, 각 후보에 대해 horizon 동안의 예측 경로를 시뮬레이션
- 실제 코드에서는 `cost = w_obs * cost_obs(path) + w_gps * cost_gps(path) + w_smooth * abs(deg - prev_opt_deg)` 형태의 비용함수를 계산
- 장애물 Cost: Cost Map 상에서 해당 경로가 얼마나 위험한지 평가
- 경로 이탈 Cost: 원래 GPS 경로에서 얼마나 벗어나는지 평가
- 제어 부드러움 Cost: 직전 최적 조향각 대비 얼마나 급격히 변하는지 평가
- 종합 비용이 가장 낮은 경로를 최적 회피 경로로 선택한 뒤 Pure Pursuit로 최종 조향 명령 생성



## 저장소 구조

```text
.
├── control
│   ├── arduino_control.py
│   ├── selector_mpc.py
│   └── gps
│       ├── gps_course_publisher.py
│       ├── mission_control.py
│       └── mpc.py
├── lane
│   ├── data
│   │   └── weights
│   │       └── best.pt
│   ├── package.xml
│   ├── setup.py
│   └── lane
│       ├── lane_custom.py
│       ├── lane_traditional.py
│       ├── path.py
│       └── highcontrol.py
└── my_lane_msgs
    ├── CMakeLists.txt
    ├── package.xml
    └── msg
        └── LanePoints.msg
```




## 결과 시각화


https://github.com/user-attachments/assets/751c6463-221e-4cab-8b5a-205b4f335e2f

