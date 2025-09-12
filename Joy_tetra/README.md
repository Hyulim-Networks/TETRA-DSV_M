# Joy_tetra

## 패키지 개요 및 역할
로봇을 조이스틱으로 조작하기 위한 ROS 패키지입니다.

## 주요 노드/스크립트 설명
- `src/joy_node.cpp`: 조이스틱 입력 처리 노드
- `launch/joy.launch`: 주요 노드 실행 런치 파일

## 설치 및 의존성
- ROS Melodic
- 의존 패키지: `ros-melodic-joy`

## 빌드 및 실행 방법
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
roslaunch Joy_tetra joy.launch
```

## 노드/토픽/서비스/액션 인터페이스
### 퍼블리시 토픽
- `/joy` (sensor_msgs/Joy): 조이스틱 입력
- `/cmd_vel` (geometry_msgs/Twist): 속도 명령

### 구독 토픽
- (해당 없음)

### 서비스
- (해당 없음)

### 액션
- (해당 없음)

## 환경 변수 및 설정 파일
- 환경 변수: 없음
- 설정 파일: `config/` 폴더 참고

## 데모/테스트 방법
```bash
# 조이스틱 노드 실행
roslaunch Joy_tetra joy.launch
# 조이스틱 입력 확인
rostopic echo /joy
# 속도 명령 확인
rostopic echo /cmd_vel
```
