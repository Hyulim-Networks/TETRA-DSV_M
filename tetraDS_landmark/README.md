# tetraDS_landmark

## 패키지 개요 및 역할
ar_tag를 인식하여 마커를 저장/표시하는 기능을 제공하는 ROS 패키지입니다.

## 주요 노드/스크립트 설명
- `src/landmark_node.cpp`: ar_tag 인식 및 마커 관리 노드
- `launch/landmark.launch`: 주요 노드 실행 런치 파일

## 설치 및 의존성
- ROS Melodic
- 의존 패키지: `ros-melodic-ar-track-alvar`, `ros-melodic-visualization-msgs`

## 빌드 및 실행 방법
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
roslaunch tetraDS_landmark landmark.launch
```

## 노드/토픽/서비스/액션 인터페이스
### 퍼블리시 토픽
- `/marker/node` (visualization_msgs/Marker): 마커 정보

### 구독 토픽
- `/ar_pose_marker` (ar_track_alvar_msgs/AlvarMarkers): AR 태그 인식 결과
- `/map_to_marker_pose` (geometry_msgs/PoseStamped): 맵-마커 위치
- `/tracked_pose` (geometry_msgs/PoseStamped): 추적된 포즈
- `/joy` (sensor_msgs/Joy): 조이스틱 입력

### 서비스
- `/savemark` (custom srv): 마커 저장

### 액션
- (해당 없음)

## 환경 변수 및 설정 파일
- 환경 변수: 없음
- 설정 파일: `config/` 폴더 참고

## 데모/테스트 방법
```bash
# 마커 노드 실행
roslaunch tetraDS_landmark landmark.launch
# AR 태그 인식 결과 확인
rostopic echo /ar_pose_marker
# 마커 저장 서비스 호출
rosservice call /savemark
```
