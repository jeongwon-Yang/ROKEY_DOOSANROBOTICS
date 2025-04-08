# 🏭 물류 자동화 디지털 트윈 시스템

## 📂 프로젝트 개요

YOLO 객체 인식과 로봇 팔 제어를 결합하여 물류 환경에서 상자를 인식하고 분류하는 자동화 시스템입니다.  
ArUco 마커 기반 내비게이션과 컨베이어 벨트 제어를 통합하여 **디지털 트윈 환경**을 구현하였으며,  
TurtleBot3 플랫폼 기반 ROS2 시뮬레이션 연동으로 **현실-가상 통합 물류 자동화**를 실현하였습니다.

---

## 📁 폴더 구조

```
04_물류자동화_디지털트윈/
├── 01_camera/               # 카메라 이미지 발행/구독
├── 02_vision/               # YOLO, ArUco 기반 물체 인식
├── 03_conveyor/             # 컨베이어 벨트 및 GUI 제어
├── 04_robot_arm/            # 로봇 팔 제어 및 테스트
├── 05_integrated_control/   # 전체 시스템 통합 제어
├── 06_turtlebot3_ws/        # TurtleBot3 시뮬레이션 및 제어 패키지
├── config/                  # 설정 파일
└── model/                   # 딥러닝 모델 파일
```

---

## 🛠 사용 기술

- **ROS2 Foxy**: 전체 통신 및 제어 시스템
- **YOLOv8 (Ultralytics)**: 실시간 객체 감지 (PyTorch)
- **OpenCV / ArUco**: 마커 추적 및 위치 계산
- **Qt + PyQt5**: 컨베이어 GUI 제어
- **TurtleBot3 + MoveIt2**: ROS 기반 로봇 시뮬레이션 및 제어
- **RViz2 / Gazebo**: 시각화 및 시뮬레이션 환경 구성

---

## 🚀 실행 순서 요약

1. **TurtleBot3 하드웨어 및 시뮬레이션**
   ```bash
   ssh -X rokey10@<robot_ip>
   ros2 launch turtlebot3_manipulation_bringup hardware.launch.py
   ros2 launch turtlebot3_manipulation_moveit_config moveit_core.launch.py
   ```

2. **YOLO 인식 & 아루코 마커 추적**
   ```bash
   python3 01_camera/compressed_image_publisher.py
   python3 02_vision/yolo_box_detector_json.py
   python3 02_vision/aruco_marker_detector.py
   ```

3. **컨베이어 벨트 GUI 제어**
   ```bash
   python3 03_conveyor/qt_conveyor_control_v2.py
   ```

4. **로봇 팔 제어 및 통합 작업 실행**
   ```bash
   python3 04_robot_arm/yolo_pick_place.py
   python3 05_integrated_control/integrated_process.py
   ```

5. **Unity와 연동 시 (옵션)**
   ```bash
   ros2 launch rosbridge_server rosbridge_websocket_launch.xml
   python3 04_robot_arm/filtered_joint_states.py
   ```

---

## 🔧 구성 요소 상세

### 📦 01_camera/
| 파일 | 설명 |
|------|------|
| `compressed_image_publisher.py` | 카메라에서 이미지를 읽어와 압축된 형태로 발행 |
| `compressed_image_viewer.py` | 발행된 이미지를 구독하여 화면에 표시 |
| `compressed_image_publisher_test.py` | 테스트용 코드 |

### 📦 02_vision/
| 파일 | 설명 |
|------|------|
| `aruco_marker_detector.py` | ArUco 마커 감지 및 위치 계산 |
| `yolo_box_detector.py` | YOLO 객체 인식 (기본) |
| `yolo_box_detector_json.py` | JSON 형식 결과 출력 |
| `yolo_object_detector.py` | 문자열 형식 감지 결과 출력 |

### 📦 03_conveyor/
| 파일 | 설명 |
|------|------|
| `conveyor_arduino_bridge.py` | ROS ↔ Arduino 브릿지 |
| `qt_conveyor_control.py` | GUI 컨트롤 기본 |
| `qt_conveyor_control_v2.py` | GUI 확장 버전 |
| `simple_qt_ros.py` | 간단한 ROS 연동 |

### 📦 04_robot_arm/
| 파일 | 설명 |
|------|------|
| `robot_arm_client.py` | 서비스 기반 제어 |
| `joint_state_filter.py` | 관절 정보 필터링 |
| `robot_arm_test.py` | 테스트용 동작 코드 |
| `robot_arm_pose_test.py` | 포즈 기반 테스트 |
| `yolo_pick_place.py` | YOLO 결과 기반 피킹 |

### 📦 05_integrated_control/
| 파일 | 설명 |
|------|------|
| `robot_task_executor.py` | GUI 기반 제어 시스템 |
| `simple_robot_control.py` | 기본 로직 통합 |
| `aruco_follower.py` | 마커 기반 추종 동작 |
| `integrated_process.py` | 전체 프로세스 통합 실행 |

### 📦 06_turtlebot3_ws/
- `turtlebot3_manipulation`: ROS2 기반 시뮬 및 제어 전체 패키지
- `turtlebot_cosmo_interface`: 커스텀 서비스 포함
- `turtlebot_moveit`: 로봇팔 제어 및 포즈 계산

### 📦 config/
- `calibration_params.yaml`: 로봇 암 조정값
- `turtlebot3_manipulation.srdf`: SRDF 설정

### 📦 model/
- `yolov8n.pt`: YOLOv8 모델 (PyTorch)

---

## ✅ .gitignore 예시

```
__pycache__/
*.pyc
*.log
.env
build/
install/
log/
*.zip
*.pt
```

---

## 📌 참고 사항

- 로봇 팔 위치 보정, ArUco 마커 기준 좌표 변환 등은 `integrated_process.py` 내부에서 자동 처리됩니다.
- Unity와 연동할 경우 ROS2 `rosbridge_server` 사용 및 odometry 토픽 필터링이 필요합니다.
- 테스트 시 ArUco 마커 및 YOLOv8 감지 모델은 정확한 환경 조정이 중요합니다.

---
