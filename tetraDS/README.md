# tetraDS

## 패키지 개요 및 역할
TETRA-DSV 구동 보드와의 Serial 통신을 통해 모터 제어를 담당하는 ROS 패키지입니다. (RS232 사용)

## 주요 노드/스크립트 설명
- `src/tetraDS_node.cpp`: 모터 제어 및 상태 수집 노드
- `launch/tetraDS.launch`: 주요 노드 실행을 위한 런치 파일

## 설치 및 의존성
- ROS Melodic
- 의존 패키지: `ros-melodic-serial`, `ros-melodic-robot-localization`

## 빌드 및 실행 방법
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
roslaunch tetraDS tetraDS.launch
```

## 노드/토픽/서비스/액션 인터페이스
### 퍼블리시 토픽
- `/odom` (nav_msgs/Odometry): 로봇 위치/자세
- `/tetra_battery` (std_msgs/Int32): 배터리 상태

### 구독 토픽
- `/cmd_vel` (geometry_msgs/Twist): 속도 명령
- `/joy` (sensor_msgs/Joy): 조이스틱 입력

### 서비스
- `/parameter_read` (custom srv): 파라미터 읽기
- `/parameter_write` (custom srv): 파라미터 쓰기
- `/mode_change` (custom srv): 모드 변경
- `/linear_position_move` (custom srv): 직선 위치 이동
- `/angular_position_move` (custom srv): 각도 위치 이동

### 액션
- (해당 없음)

## 환경 변수 및 설정 파일
- 환경 변수: 없음
- 설정 파일: `config/` 폴더 내 YAML 파일 참고

## 데모/테스트 방법
```bash
# 속도 명령 테스트
rostopic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.1}}"
# 오도메트리 확인
rostopic echo /odom
# 배터리 상태 확인
rostopic echo /tetra_battery
# 서비스 테스트(예시)
rosservice call /parameter_read
rosservice call /mode_change
```
