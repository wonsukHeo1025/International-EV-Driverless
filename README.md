# 제4회 국제 대학생 EV 자율주행 대회 1/5부문

![KakaoTalk_20250719_171758370](https://github.com/user-attachments/assets/b36175fc-826d-460e-8fbc-186d31eb8bc5)


## 개요

담당: lane 패키지 전체 + my_lane_msgs 패키지 전체 + GPS 절대 경로와 LiDAR 상대 객체를 같은 좌표계에 align하는 로직

이 README는 직접 구현한 [`lane`](./lane) 패키지 전체, [`my_lane_msgs`](./my_lane_msgs) 패키지 전체, 그리고 [`control/gps/mission_control.py`](./control/gps/mission_control.py) 내 GPS-LiDAR 좌표계 통합 로직을 설명하기 위한 문서입니다.  
`point_cloud_processor`, `mpc.py`, `selector_mpc.py`, Arduino 제어부는 프로젝트에는 포함되지만 제 담당 범위는 아니므로 여기서는 제 모듈과 맞물리는 인터페이스 중심으로만 설명합니다.

## 담당 범위

| 영역 | 패키지/모듈 | 구현한 내용 |
|---|---|---|
| Lane Perception | [`lane`](./lane) | 카메라 기반 차선 검출, BEV 변환, DBSCAN/RANSAC/Sliding Window/Kalman 기반 차선 포인트 추출, 차선 중심 경로 생성, Pure Pursuit 기반 차선 조향각 계산 |
| ROS 2 Interface | [`my_lane_msgs`](./my_lane_msgs) | 좌/우 차선 좌표 전달을 위한 `LanePoints.msg` 인터페이스 설계 및 패키지 구성 |
| Frame Integration | [`control/gps/mission_control.py`](./control/gps/mission_control.py) | GPS 절대 좌표 waypoint를 로컬 XY로 변환하고 차량 기준 상대 좌표로 재투영하여, LiDAR 클러스터 객체와 동일한 `velodyne` 기준 좌표계에서 경로를 시각화 및 활용 가능하도록 구성 |

## 시스템 목표

제가 맡은 모듈 기준으로 이 시스템은 다음 흐름을 수행하도록 설계되었습니다.

Lane
- 전방 카메라 영상에서 좌/우 차선을 추출
- 차선 점들을 기반으로 중앙 주행 경로를 생성
- 생성된 경로에서 look-ahead target을 선택해 조향각을 계산
- 차선 검출 여부를 함께 발행해 lane 주행과 GPS 주행 전환에 활용

GPS-LiDAR Frame Integration
- CSV 기반 GPS 절대 경로를 로컬 XY 좌표계로 변환
- GNSS 위치와 course heading을 이용해 waypoint를 차량 기준 상대 좌표로 변환
- 변환된 경로를 velodyne 프레임 기준 /gps_path로 발행
- LiDAR에서 검출된 상대 좌표 장애물 마커와 같은 좌표계에서 경로를 겹쳐 표시하고 MPC 입력으로 연결



## 주요 기능

### 1. Lane 패키지 전체 구현

- 커스텀 YOLO 기반 차선 세그멘테이션 파이프라인 구현
- 전통적 영상처리 기반 차선 검출 파이프라인 구현
- Bird's Eye View 변환으로 차선 geometry를 정규화
- DBSCAN, RANSAC, Kalman filter 기반으로 노이즈와 순간 검출 실패를 완화
- 좌/우 차선이 모두 있을 때는 중앙선 생성
- 한쪽 차선만 존재할 때는 offset 기반으로 가상 중심 경로 복원
- spline 기반 재샘플링으로 제어용 path 생성
- Pure Pursuit 기반 조향각 계산

### 2. 커스텀 메시지 인터페이스 구현

- 좌/우 차선 포인트를 각각 left_x, left_y, right_x, right_y 배열로 전달

### 3. GPS 경로와 LiDAR 객체의 동일 좌표계 정렬

- GNSS waypoint CSV를 기준으로 origin 설정
- 위도/경도를 로컬 XY로 변환해 절대 경로를 평면 좌표계로 투영
- 실시간 GNSS 위치와 heading을 사용해 waypoint를 차량 기준 상대 좌표로 변환
- 변환된 GPS 경로를 토픽으로 퍼블리시
- 변환된 GPS 경로 토픽의 frame_id 를 velodyne으로 맞춰 LiDAR 클러스터 마커와 동일 좌표계에 배치
- 결과적으로 RViz와 MPC에서 전역 경로와 상대 장애물을 동시에 다룰 수 있도록 처리



## 저장소 구조

```text
.
├── control
│   └── gps
│       ├── mission_control.py
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

## 기술 스택

- ROS 2 Humble
- Python
- OpenCV
- Ultralytics YOLO
- ROS 2 Custom Message Interface
- GNSS local coordinate transform
- LiDAR-RViz frame visualization


## 결과 시각화


https://github.com/user-attachments/assets/751c6463-221e-4cab-8b5a-205b4f335e2f

