# tetraDS_service

## 패키지 개요 및 역할
환경지도, 자율주행 등 다양한 기능을 서비스 형태로 제공하는 ROS 패키지입니다.

## 주요 노드/스크립트 설명
- `src/service_node.cpp`: 서비스 제공 노드
- `launch/service.launch`: 주요 노드 실행 런치 파일

## 설치 및 의존성
- ROS Melodic
- 의존 패키지: `ros-melodic-navigation`, `ros-melodic-move-base`

## 빌드 및 실행 방법
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
roslaunch tetraDS_service service.launch
```

## 노드/토픽/서비스/액션 인터페이스
### 퍼블리시 토픽
- `/map` (nav_msgs/OccupancyGrid): 맵 데이터
- `/odom` (nav_msgs/Odometry): 오도메트리 데이터

### 구독 토픽
- `/cmd_vel` (geometry_msgs/Twist): 속도 명령
- 기타 센서/상태 토픽

### 액션
- `/move_base` (move_base_msgs/MoveBaseAction): 네비게이션 액션

### 서비스
- `/tetraDS_service/map_save` (std_srvs/Empty): 맵 저장
- `/tetraDS_service/start_navigation` (std_srvs/Empty): 네비게이션 시작
- 기타 서비스: 환경설정, 경로 요청 등

## 환경 변수 및 설정 파일
- 환경 변수: 없음
- 설정 파일: `config/` 폴더 참고

## 데모/테스트 방법
```bash
# 서비스 노드 실행
roslaunch tetraDS_service service.launch
# 맵 저장 서비스 호출
rosservice call /tetraDS_service/map_save
# 네비게이션 시작 서비스 호출
rosservice call /tetraDS_service/start_navigation
# 맵/오도메트리/액션 확인
rostopic echo /map
rostopic echo /odom
```
