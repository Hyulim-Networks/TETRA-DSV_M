# tetraDS_2dnav

## 패키지 개요 및 역할
네비게이션 관련 설정, 런치, 맵, 스크립트 등을 관리하는 패키지입니다.

## 주요 노드/스크립트 설명
- `launch/navigation.launch`: 네비게이션 전체 실행 런치 파일
- `scripts/`: 네비게이션 관련 스크립트

## 설치 및 의존성
- ROS Melodic
- 의존 패키지: `ros-melodic-navigation`, `ros-melodic-move-base`, `ros-melodic-map-server`

## 빌드 및 실행 방법
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
roslaunch tetraDS_2dnav navigation.launch
```

## 노드/토픽/서비스/액션 인터페이스
### 퍼블리시 토픽
- `/map` (nav_msgs/OccupancyGrid): 맵 데이터
- `/robot_global_pose` (geometry_msgs/PoseStamped): 로봇 전역 위치

### 구독 토픽
- `/tf_listener` (nav_msgs/Odometry): 오도메트리
- 기타 센서/상태 토픽

### 액션
- `/move_base` (move_base_msgs/MoveBaseAction): 네비게이션 액션

### 서비스
- (해당 없음)

## 환경 변수 및 설정 파일
- 환경 변수: 없음
- 설정 파일: `config/`, `maps/` 폴더 참고

## 데모/테스트 방법
```bash
# 네비게이션 노드 실행
roslaunch tetraDS_2dnav navigation.launch
# 맵/로봇 위치/오도메트리 확인
rostopic echo /map
rostopic echo /robot_global_pose
rostopic echo /tf_listener
```
