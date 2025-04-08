# 🤝 협동 로봇 정찰 및 구조 시스템

## 📂 프로젝트 개요

Gazebo 시뮬레이션 기반으로 TurtleBot3과 매니퓰레이터 로봇을 활용하여  
다중 로봇 환경에서 자율 정찰 및 구조 임무를 수행하는 시스템을 구현했습니다.  
SLAM 기반 지도 생성부터 GUI 기반 구조 명령 실행까지 전체 로봇 제어를 통합하였습니다.

## 📁 폴더 구조

07_협동로봇정찰_구조시스템/
├── 01_multi_robot_nav/           # 다중 로봇 내비게이션 설정
│   └── multi_nav2_world.launch.py      # TurtleBot3 SLAM + Nav2 실행 런치 파일
│
├── 02_integrated_system/         # 구조 로봇 통합 제어 시스템
│   ├── integrated_simulation_launcher.py   # 구조 로봇 시뮬 전체 실행
│   └── integrated_gui_control.py          # PyQt GUI 기반 구조 명령 제어

## ⚙️ 실행 환경

- Ubuntu 22.04
- ROS2 Humble
- Python 3.8+
- Gazebo 11
- PyQt5, rclpy, nav2, slam_toolbox 등

## 🔧 설치 방법

```bash
sudo apt update
sudo apt install ros-humble-navigation2 ros-humble-slam-toolbox
pip install pyqt5
```

## 🚀 실행 순서

1. SLAM 기반 다중 로봇 내비게이션 실행

```bash
ros2 launch 01_multi_robot_nav/multi_nav2_world.launch.py
```

2. 구조 로봇 GUI 기반 통합 제어 실행

```bash
python3 02_integrated_system/integrated_simulation_launcher.py
```

3. GUI 제어 패널 실행

```bash
python3 02_integrated_system/integrated_gui_control.py
```

## 🧠 주요 기능 요약

- TurtleBot3 기반 SLAM 정찰 및 경로 계획
- GUI 기반 구조 명령(구조 시작 / 배터리 모니터링 / 로봇 상태 표시)
- 매니퓰레이터 로봇을 활용한 구조 행동 시뮬레이션
- PyQt5를 통한 실시간 사용자 인터페이스 제공
- 다중 로봇 상태 및 경로 정보 통합 관리

## ✅ .gitignore 예시

```gitignore
__pycache__/
*.pyc
*.log
.env
build/
install/
log/
```

---
