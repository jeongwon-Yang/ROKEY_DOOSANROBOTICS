# 🤖 협동 로봇 제어 응용 실습

## 📂 프로젝트 개요

Doosan 협동로봇 M0609를 활용하여 기본적인 무브J/L 제어부터  
팔레트 자동 정렬, 기어 조립, 실시간 디지털 젠가 시계 구현까지  
다양한 자동화 응용 실습을 수행하였습니다.

---

## 📁 폴더 구조

```
03_협동로봇제어_응용실습/
├── basic/                        # 무브J/L 기반 Pick & Place, 기본 제어
│   ├── force_pick_place.drl     # Force 기반 피킹 & 플레이스
│   └── CollaborativeRobot2_E1_Sourcecode_backup.py
├── advanced/                     # 응용 과제
│   ├── height_based_sorting.py  # 블록 높이 기반 자동 정렬 시스템
│   ├── jenga_digital_clock.py   # 실시간 디지털 젠가 시계 구현
│   └── m0609_assemble_gear.drl  # 기어 자동 조립 시스템
├── config/
│   └── 9815.json                 # Doosan 2FG 그리퍼 설정 JSON
```

---

## ⚙️ 실행 환경

- Doosan Robotics M0609
- DSR SDK (Python API)
- ROS2 rclpy (옵션)
- Python 3.8+

> 💡 `9815.json` 파일은 Doosan Teach Pendant에서 사용되는  
그리퍼(Tool Weight_2FG) 설정 파일이며, 환경 재현 시 참고합니다.

---

## 🔧 설치 방법

```bash
pip install rclpy
# Doosan SDK 제공 환경에서 실행 필요
```

---

## 🚀 주요 실습 예시

### 1. 무브J / 무브L 기반 Pick & Place
- 팔레트 간 블록을 일정 경로로 옮기는 기본 실습
- .drl 또는 Python 코드 기반 구현

### 2. 블록 높이 기반 자동 분류 시스템
- Force 센서 기반 블록 높이 측정
- 긴/중간/짧은 블록을 자동 분류하여 좌측 팔레트로 이동
- 그리퍼 제어 + Compliance Control + 위치 계산

### 3. 디지털 젠가 시계
- 실시간 시계를 젠가 블록 배열로 표현
- 세그먼트 A~G 기반 숫자 표현
- 필요 없는 블록만 제거하며 현재 시간을 표시

### 4. 기어 자동 조립 시스템
- 3개의 큰 기어 + 1개의 중심 기어를 자동으로 조립
- 위치 계산 기반 pick → place 반복 수행

---

## 🛠 사용 기술 요약

- DSR SDK: 로봇 제어 API
- movej / movel: 관절 및 선형 이동 제어
- Force Compliance Control: 접촉 기반 높이 측정
- Enum 기반 세그먼트 제어: 디지털 시계 구현
- Python 기반 실시간 작업 로직
- JSON 기반 그리퍼 세팅 관리 (9815.json)

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
```
```

---
