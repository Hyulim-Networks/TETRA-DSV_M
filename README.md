# TETRA-DSV_M

![1](https://user-images.githubusercontent.com/103166594/220823974-8fde85da-4c52-4bb3-9638-9e6d5bca1c39.png)

## Hyulimnetworks TETRA-DSV ROS Melodic Mobile Robot Platform (Monorepo)

---

### 목차
- [TETRA-DSV\_M](#tetra-dsv_m)
  - [Hyulimnetworks TETRA-DSV ROS Melodic Mobile Robot Platform (Monorepo)](#hyulimnetworks-tetra-dsv-ros-melodic-mobile-robot-platform-monorepo)
    - [목차](#목차)
    - [모노레포 구조 및 패키지 설명](#모노레포-구조-및-패키지-설명)
    - [지원 환경 및 ROS 버전](#지원-환경-및-ros-버전)
    - [설치 및 빌드 방법](#설치-및-빌드-방법)
      - [ROS Melodic 설치](#ros-melodic-설치)
      - [프로젝트 클론 및 의존성 설치](#프로젝트-클론-및-의존성-설치)
      - [빌드 및 실행](#빌드-및-실행)
    - [주요 기능 및 데모](#주요-기능-및-데모)
    - [기여 가이드](#기여-가이드)
    - [FAQ 및 문제 해결](#faq-및-문제-해결)
    - [라이선스 및 연락처](#라이선스-및-연락처)

---

### 모노레포 구조 및 패키지 설명

본 프로젝트는 ROS Melodic 기반 모바일 로봇 플랫폼의 모든 ROS 패키지를 하나의 모노레포로 관리합니다.
각 폴더/패키지의 주요 역할은 다음과 같습니다:

- **tetraDS** : 구동 보드와의 Serial 통신을 통한 모터 제어 (RS232)
- **tetraDS_description** : 로봇 URDF 및 모델 파일
- **tetraDS_interface** : 전원공급, LED/GPIO 제어 (RS232)
- **tetraDS_landmark** : ar_tag 기반 마커 저장/표시
- **tetraDS_TCP** : 외부 디바이스와 ROS 간 소켓 통신
- **tetraDS_service** : 환경지도, 자율주행 등 서비스 제공
- **tetraDS_2dnav** : 설정/런치/맵/스크립트 등 네비게이션 관련
- **virtual_costmap_layer, virtual_costmap_layer2** : 글로벌/로컬 costmap용 가상 레이어
- **Joy_tetra** : 조이스틱 제어
- **async_web_server_cpp** : C++ 비동기 웹/웹소켓 서버
- **web_video_server** : ROS 이미지 토픽 HTTP 스트리밍
- **cyglidar_d1-0.2.0** : ToF 라이다 드라이버
- **iahrs_driver-master** : iAHRS IMU 센서 드라이버
- **realsense-ros** : Intel RealSense 카메라 ROS 패키지
- **sick_tim-kinetic** : SICK TiM/MRS1000 레이저 스캐너 드라이버
- **usb_cam-develop** : USB 카메라 ROS 드라이버
- **sh_files** : 전체 구동용 쉘 스크립트

---

### 지원 환경 및 ROS 버전

- **OS**: Ubuntu 18.04
- **ROS**: Melodic (권장)
- 기타 의존성 및 패키지는 아래 설치 방법 참고

> **주의:** 시그봇 라이다 패키지는 D2 모델 기준이며, D1 모델과 호환되지 않습니다.

---

### 설치 및 빌드 방법

#### ROS Melodic 설치
```bash
# source.list 설정
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
# keys 설정
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
# 패키지 목록 최신화
sudo apt update
# ROS melodic 설치
sudo apt install ros-melodic-desktop-full
# 환경변수 설정
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
# 의존성 패키지 설치
sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
# rosdep 초기화
sudo rosdep init
rosdep update
```

#### 프로젝트 클론 및 의존성 설치
```bash
mkdir ~/catkin_ws
cd ~/catkin_ws
git clone https://github.com/Hyulim-Networks/TETRA-DSV_M.git
mv ~/catkin_ws/TETRA-DSV_M ~/catkin_ws/src
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
# 주요 ROS 패키지 설치
sudo apt-get install ros-melodic-move-base ros-melodic-navigation ros-melodic-serial ros-melodic-rosbridge-server \
		ros-melodic-teb-local-planner ros-melodic-spatio-temporal-voxel-layer ros-melodic-ar-track-alvar \
		ros-melodic-rgbd-launch ros-melodic-tf2-web-republisher ros-melodic-robot-localization ros-melodic-joy rapidjson-dev
# 시리얼 통신 권한 설정
sudo chmod 666 /dev/ttyS0
sudo chmod 666 /dev/ttyS1
```

#### 빌드 및 실행
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
# 예시: 주요 런치 파일 실행
roslaunch tetraDS_2dnav navigation.launch
```

---

### 주요 기능 및 데모

- SLAM 및 자율주행
- 실시간 센서 데이터 처리 및 시각화
- 다양한 센서(라이다, 카메라, IMU 등) 연동
- 웹 기반 모니터링/제어
- 조이스틱/원격 제어

> 데모 영상/스크린샷은 각 패키지 폴더의 README 또는 `/screenshots` 폴더 참고

---

### 기여 가이드

1. 이슈 등록 및 토론 후 브랜치 생성
2. 기능/버그별 브랜치에서 작업 후 PR 요청
3. 코드 스타일 및 문서화 권장
4. 주요 변경사항은 README에 반영

---

### FAQ 및 문제 해결

- **Q: 빌드 오류 발생 시?**
	- ROS 및 의존성 패키지 설치 여부 확인
	- `rosdep install --from-paths src --ignore-src -r -y` 재실행
- **Q: 시리얼 통신 오류?**
	- 권한 설정(`sudo chmod 666 /dev/ttyS*`) 확인
- **Q: 센서 인식 안됨?**
	- 연결 상태 및 드라이버 설치 확인

---

### 라이선스 및 연락처

- **라이선스**: MIT
- **문의/기여/이슈**: [GitHub Issues](https://github.com/Hyulim-Networks/TETRA-DSV_M/issues) 또는 hyulimnetworks 연구소 담당자 이메일 jyahn@oneulenm.com
