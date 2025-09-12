# tetraDS_interface

## 패키지 개요 및 역할
TETRA-DSV의 전원공급, LED, GPIO 등 보드 제어를 위한 ROS 패키지입니다. (RS232 사용)

## 주요 노드/스크립트 설명
- `src/interface_node.cpp`: 보드 제어 및 상태 노드
- `launch/interface.launch`: 주요 노드 실행 런치 파일

## 설치 및 의존성
- ROS Melodic
- 의존 패키지: `ros-melodic-serial`

## 빌드 및 실행 방법
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
roslaunch tetraDS_interface interface.launch
```

## 노드/토픽/서비스/액션 인터페이스
### 퍼블리시 토픽
- `/GPIO` (tetraDS_interface/GPIO): GPIO 상태

### 구독 토픽
- `/led_control` (std_msgs/Bool): LED 제어

### 서비스
- `/chargeport_on` (custom srv): 충전포트 ON
- `/chargeport_off` (custom srv): 충전포트 OFF
- `/led` (custom srv): LED 제어
- `/ledtoggle` (custom srv): LED 토글
- `/turnon` (custom srv): 전원 ON
- `/log` (custom srv): 로그 요청
- `/power_outport` (custom srv): 파워 아웃포트 제어
- `/power_single_outport` (custom srv): 단일 아웃포트 제어
- `/power_get_io` (custom srv): IO 상태 요청
- `/loadcell_callibration` (custom srv): 로드셀 캘리브레이션

### 액션
- (해당 없음)

## 환경 변수 및 설정 파일
- 환경 변수: 없음
- 설정 파일: `config/` 폴더 참고

## 데모/테스트 방법
```bash
# LED 제어 테스트
rostopic pub /led_control std_msgs/Bool "data: true"
# GPIO 상태 확인
rostopic echo /GPIO
# 서비스 테스트(예시)
rosservice call /chargeport_on
rosservice call /ledtoggle
```
