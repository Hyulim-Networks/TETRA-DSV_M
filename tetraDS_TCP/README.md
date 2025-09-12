# tetraDS_TCP

## 패키지 개요 및 역할
외부 디바이스와 ROS 간의 다이렉트 소켓 통신을 위한 패키지입니다.

## 주요 노드/스크립트 설명
- `src/tcp_node.cpp`: TCP 통신 노드
- `launch/tcp.launch`: 주요 노드 실행 런치 파일

## 설치 및 의존성
- ROS Melodic
- 의존 패키지: `ros-melodic-rosbridge-server`

## 빌드 및 실행 방법
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
roslaunch tetraDS_TCP tcp.launch
```

## 노드/토픽/서비스/액션 인터페이스
### 퍼블리시 토픽
- `/tcp/data_out` (std_msgs/String): ROS에서 외부로 송신하는 데이터
- `/odom` (nav_msgs/Odometry): 오도메트리 데이터

### 구독 토픽
- `/tcp/data_in` (std_msgs/String): 외부 디바이스에서 수신한 데이터
- `/cmd_vel` (geometry_msgs/Twist): 속도 명령
- 기타 센서/상태 토픽

### 액션
- `/move_base` (move_base_msgs/MoveBaseAction): 네비게이션 액션

### 서비스
- (해당 없음)

## 환경 변수 및 설정 파일
- 환경 변수: 없음
- 설정 파일: `config/` 폴더 참고

## 데모/테스트 방법
```bash
# TCP 노드 실행
roslaunch tetraDS_TCP tcp.launch
# 외부로 데이터 송신 테스트
rostopic pub /tcp/data_out std_msgs/String "data: 'test'"
# 수신 데이터 확인
rostopic echo /tcp/data_in
# 오도메트리/속도 명령 확인
rostopic echo /odom
rostopic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.1}, angular: {z: 0.1}}"
```
