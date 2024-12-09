# TETRA-DSV_M

![1](https://user-images.githubusercontent.com/103166594/220823974-8fde85da-4c52-4bb3-9638-9e6d5bca1c39.png)

####   Hyulimnetworks TETRA-DSV ROS Dev

************************

## 설치방법
- 우분투 18.04 설치


- ROS melodic 설치

``` bash
# source.list 설정
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# keys 설정
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# 패키지 목록을 최신화 
sudo apt update

# ROS melodic 설치
sudo apt install ros-melodic-desktop-full

# 환경변수 설정
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc

# 의존성 패키지 빌드 설치
sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential

# rosdep 초기화
sudo rosdep init
rosdep update

# tetra 패키지 설치 및 의존성 패키지 설치
mkdir ~/catkin_ws
cd ~/catkin_ws
git clone https://github.com/Hyulim-Networks/TETRA-DSV_M.git
mv ~/catkin_ws/TETRA-DSV_M ~/catkin_ws/src
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
sudo apt-get install ros-melodic-move-base
sudo apt-get install ros-melodic-navigation
sudo apt-get install ros-melodic-serial
sudo apt-get install ros-melodic-rosbridge-server
sudo apt-get install ros-melodic-teb-local-planner
sudo apt-get install ros-melodic-spatio-temporal-voxel-layer
sudo apt-get install ros-melodic-ar-track-alvar
sudo apt-get install ros-melodic-rgbd-launch
sudo apt-get install ros-melodic-tf2-web-republisher
sudo apt-get install ros-melodic-robot-localization
sudo apt-get install ros-melodic-joy
sudo apt install rapidjson-dev

# 시리얼 통신 권한 설정
sudo chmod 666 /dev/ttyS0
sudo chmod 666 /dev/ttyS1

```

- 패키지 설치
- 프로젝트 빌드
- 프로젝트 실행

************************

## 패키지 설명

#### tetraDS : TETRA-DSV구동 보드와의 Serial통신을 통하여 Motor의 구동을 담당하는 패키지 (RS232통신 사용)

#### tetraDS_description : TETRA-DSV의 URDF가 포함된 패키지 (gazebo 지원 X)

#### tetraDS_interface : TETRA-DSV의 전원공급, LED/GPIO 등의 제어를 제공하는 보드 사용 패키지 (RS232통신 사용)

#### tetraDS_landmark : 인식되는 ar_tag를 이용하여 마커를 저장/표시하기 위한 패키지

#### tetraDS_TCP : 외부 디바이스와 ROS 간의 다이렉트 Socket통신을 위하여 제작한 통신용 패키지

#### tetraDS_service : TETRA-DSV의 환경지도/자율주행/각종 기능들에 서비스를 사용하기 위한 패키지

#### tetraDS_2dnav : TETRA-DSV의 각종 설정 파일과 런치 파일들을 모아둔 패키지

#### virtual_costmap_layer : GMahmoud가 만든 가상 레이어 패키지 (Global costmap에 활용)

#### virtual_costmap_layer2 : GMahmoud가 만든 가상 레이어 패키지에서 플러그인 이름만 수정한 버전 (local costmap에 활용)

#### Joy_tetra : 로봇을 조이스틱으로 조작하기 위한 패키지

#### async_web_server_cpp : C++로 만든 비동기 웹/웹소켓 서버

#### web_video_server : 다양한 포맷의 ROS 이미지 토픽의 HTTP 스트리밍 패키지

#### cyglidar_d1-0.2.0 : ToF 방식의 시그봇 라이다 패키지

#### iahrs_driver-master : intelliThings iAHRS IMU(3축 자이로, 가속도, 지자기 센서 통합 AHRS) 패키지

#### realsense-ros : Intel(R) RealSense(TM) 카메라용 ROS 패키지

#### sick_tim-kinetic : SICK TiM 및 SICK MRS 1000 레이저 스캐너용 ROS 드라이버

#### usb_cam-develop : V4L USB 카메라용 ROS 드라이버

#### sh_files : 로봇의 전체적인 구동을 위한 쉘 스크립트 실행파일

<br>

