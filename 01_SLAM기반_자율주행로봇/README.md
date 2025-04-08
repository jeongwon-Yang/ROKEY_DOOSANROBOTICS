# 🧭 SLAM 기반 자율주행 청소 로봇 시스템

## 개요
TurtleBot4와 ROS2를 이용하여 실내 환경에서 자율적으로 SLAM을 수행하고,  
맵 생성 후 자동으로 청소 모드로 전환되는 로봇 시스템을 구현하였습니다.

## 📁 폴더 구조
```
01_SLAM기반_자율주행로봇/
├── cleaner/              # 자율주행 및 청소 알고리즘 Python 코드
│   ├── mapping.py
│   ├── main_cleaner.py
│   ├── robot_path.csv
│   └── robot_path.png
├── config/               # SLAM 및 Nav2 관련 설정
│   ├── nav2.yaml
│   └── slam.yaml
├── map/                  # 맵 데이터 결과물
│   ├── turtlebot4_map.pgm
│   └── turtlebot4_map.yaml
```

## 🚀 실행 순서

1. SLAM 맵핑 수행  
```bash
ros2 run turtlebot_exploration_pkg mapping.py
```

2. 맵 저장 후, 청소 모드 진입  
```bash
ros2 run turtlebot_exploration_pkg main_cleaner.py
```

## 🛠 사용 기술
- ROS2 (rclpy)
- Nav2
- OccupancyGrid, OpenCV
- BFS 알고리즘 (청소 경로 탐색)
```

---

## ✅ `requirements.txt`

```txt
rclpy
opencv-python
numpy
matplotlib
```

---

## ✅ `.gitignore`

```gitignore
__pycache__/
*.pyc
*.db3
*.log
*.rviz
*.bag
*.launch.py~
*.swp
.env
build/
install/
log/
```

---
