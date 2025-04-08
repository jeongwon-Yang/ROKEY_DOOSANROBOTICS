# 🤖 서빙 로봇 서비스 시스템

---

## 📂 프로젝트 개요

PyQt 기반의 키오스크에서 주문이 이루어지면,  
ROS2 통신을 통해 주방으로 주문이 전달되고,  
서빙 로봇은 Nav2 기반 자율 주행을 통해 테이블까지 이동하는 자동화 서빙 시스템입니다.

---

## 📁 폴더 구조

```
02_서빙로봇_서비스/
├── kiosk/                  # 주문 및 통계 처리
│   ├── main.py                 # 키오스크 UI 구현
│   ├── publish_order.py        # 주문 퍼블리셔 (ROS2)
│   ├── statistics.py           # 주문 내용 DB 저장
│   └── order_service.srv       # 주문 서비스 메시지 포맷
├── kitchen/                # 주방 응답 처리
│   └── kitchen_server.py       # 주문 수신 및 처리 UI
```

---

## ⚙️ 실행 환경

- Python 3.8+
- ROS2 (Foxy / Humble)
- PyQt5, rclpy

---

## 🔧 설치 방법

1. 의존성 설치

```bash
pip install pyqt5
pip install -r requirements.txt  # 필요 시 별도 작성
```

2. ROS2 워크스페이스 설정 및 빌드 (예시)

```bash
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

---

## 🚀 실행 순서

1. 키오스크 UI 실행

```bash
python3 kiosk/main.py
```

2. 주문 발생 시 퍼블리셔가 ROS2 서비스 요청 송신  
3. kitchen_server.py 실행 (주방 응답 시스템)

```bash
python3 kitchen/kitchen_server.py
```

4. 이후 로봇 자율 주행 시스템과 연결하여 테이블까지 서빙 가능

---

## 🛠 사용 기술 요약

- **PyQt5**: 키오스크 및 주방 UI 구성
- **ROS2**: 퍼블리셔/서브스크라이버 및 서비스 메시지 통신
- **rclpy**: Python 기반 ROS2 노드 구현
- **Nav2 연동 가능**: 서빙 로봇 경로 설정 및 목표 테이블 주행

---

## 🧠 주요 기능 흐름 요약

1. 키오스크에서 메뉴 선택 → 주문 발생  
2. 주문 정보는 ROS2 퍼블리셔를 통해 송신  
3. 주방 서버가 주문을 수신하여 확인  
4. 향후 로봇은 자율 주행을 통해 서빙

---

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

