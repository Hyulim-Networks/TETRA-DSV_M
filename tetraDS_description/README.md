# tetraDS_description

## 패키지 개요 및 역할
TETRA-DSV 로봇의 URDF 및 3D 모델을 제공하는 패키지입니다. (gazebo 미지원)

## 주요 노드/스크립트 설명
- 모델 파일: `urdf/tetraDS.urdf.xacro`
- RViz 설정: `rviz/tetraDS.rviz`

## 설치 및 의존성
- ROS Melodic
- 의존 패키지: `ros-melodic-xacro`, `ros-melodic-rviz`

## 빌드 및 실행 방법
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
roslaunch tetraDS_description view_model.launch
```

## 노드/토픽/서비스/액션 인터페이스
- `/robot_description` (std_msgs/String): URDF 데이터

## 환경 변수 및 설정 파일
- 환경 변수: 없음
- 설정 파일: `urdf/`, `rviz/` 폴더 참고

## 데모/테스트 방법
```bash
roslaunch tetraDS_description view_model.launch
```
