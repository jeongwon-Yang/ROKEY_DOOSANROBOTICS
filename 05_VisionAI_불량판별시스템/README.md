# 🧠 Vision AI 기반 불량 판별 시스템

> 📄 전체 프로젝트 소개 및 과정은 [Notion 포트폴리오 보기](https://www.notion.so/Raspberry-Pi-YOLOv6-1cd9c55c6631801ab169ebca41582d07) 를 참고해주세요.

---

## 📂 프로젝트 개요

Raspberry Pi 기반의 컨베이어 벨트 시스템 위에서  
YOLOv6 API를 통해 부품 불량 여부를 실시간 판정하고,  
Google Sheets/Drive API로 결과를 자동 기록 및 시각화하는 시스템입니다.

---

## ⚙️ 실행 환경

- Python 3.8+
- Raspberry Pi (or Linux Ubuntu)
- OpenCV, gspread, PyDrive2, oauth2client, requests 등 사용

---

## 🔧 설치 방법

### 1. 의존성 설치
```bash
pip install -r requirements.txt

2. JSON 키 파일 등록

    main.py 안의 이 부분 수정 필요:

JSON_KEYFILE = r"/your/path/to/your-service-account.json"

🚀 실행 방법

python main.py

    컨베이어에서 제품이 감지되면 카메라 촬영 → YOLO API로 전송

    결과에 따라 Google Sheet 기록 및 불량 이미지는 Google Drive에 업로드됨

    라즈베리파이와 아두이노는 시리얼 통신을 통해 제품 정지/재가동 제어

🧾 파일 설명
파일	설명
main.py	전체 시스템 구현 코드 (센서 → 추론 → 기록까지)
requirements.txt	실행에 필요한 패키지 리스트
defective_images/	불량 판정 이미지 저장 폴더 (자동 생성됨)
❓ 참고사항

    YOLO API는 Superb AI Suite 엔드포인트로 호출됩니다.

    프로젝트의 목적, 설계 배경, 모델 성능 분석은 Notion 포트폴리오에 상세히 정리되어 있습니다.

🧠 주요 기능 흐름 요약

    센서 감지

    카메라 이미지 캡처

    YOLOv6 API 추론 요청

    결과 기반 양/불 판별

    Google Sheets 기록

    불량일 경우 Drive에 이미지 업로드 및 링크 저장

    아두이노 시리얼로 컨베이어 정지/재가동 신호 전송
