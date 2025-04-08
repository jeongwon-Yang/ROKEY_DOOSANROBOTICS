import cv2
import numpy as np
import math
from ultralytics import YOLO
import os

# 글로벌 변수: 각 객체의 persistent ID와 해당 객체의 re‑ID 특징(a, b 채널) 저장
tracked_objects = {}  # { id: (a, b) }
next_id = 1
COLOR_THRESHOLD = 20  # a, b 채널 간 유클리드 거리가 이 값 이하이면 동일 객체로 간주 (조정 가능)

def get_object_id(avg_ab):
    global next_id, tracked_objects
    # 기존에 저장된 객체와 비교하여 re‑ID 매칭
    for obj_id, stored_ab in tracked_objects.items():
        distance = math.sqrt(
            (avg_ab[0] - stored_ab[0])**2 +
            (avg_ab[1] - stored_ab[1])**2
        )
        if distance < COLOR_THRESHOLD:
            # (옵션) 기존 값과 현재 값을 평균으로 업데이트
            updated_ab = (
                int((stored_ab[0] + avg_ab[0]) / 2),
                int((stored_ab[1] + avg_ab[1]) / 2)
            )
            tracked_objects[obj_id] = updated_ab
            return obj_id
    # 유사 객체가 없다면 새 ID 부여
    obj_id = next_id
    next_id += 1
    tracked_objects[obj_id] = avg_ab
    return obj_id

def run_tracking(model):
    cap = cv2.VideoCapture(0)  # 카메라 인덱스 (환경에 맞게 조정)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cv2.namedWindow("Tracking", cv2.WINDOW_NORMAL)
    
    while True:
        ret, frame = cap.read()
        if not ret:
            print("프레임을 가져올 수 없습니다.")
            break
        
        # YOLO 모델로 객체 검출 (stream=True 사용)
        results = model(frame, stream=True)
        
        for r in results:
            for box in r.boxes:
                # 바운딩 박스 좌표 추출 및 정수 변환
                x1, y1, x2, y2 = box.xyxy[0]
                x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
                
                # ROI 추출 및 평균 색상 계산 (먼저 BGR, 나중에 Lab로 변환)
                roi = frame[y1:y2, x1:x2]
                if roi.size == 0:
                    continue
                
                # ROI를 Lab 색상 공간으로 변환
                lab_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2LAB)
                avg_lab = cv2.mean(lab_roi)[:3]  # L, a, b 값
                # re‑ID 비교를 위해 a, b 채널만 사용
                avg_ab = (int(avg_lab[1]), int(avg_lab[2]))
                
                # persistent re‑ID 매칭: 기존 객체와 유사하면 같은 ID, 아니면 새 ID 부여
                obj_id = get_object_id(avg_ab)
                
                # 화면 표시용: 원래 ROI의 평균 색상을 BGR에서 RGB로 변환
                avg_bgr = cv2.mean(roi)[:3]
                avg_rgb = (int(avg_bgr[2]), int(avg_bgr[1]), int(avg_bgr[0]))
                # R 채널 기준: R 값이 120 이상이면 녹색, 아니면 파란색
                box_color = (0, 0, 255) if avg_rgb[0] >= 100 else (255, 0, 0)
                
                cv2.rectangle(frame, (x1, y1), (x2, y2), box_color, 2)
                cv2.putText(frame, f"ID: {obj_id}", (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, box_color, 2)
        
        cv2.imshow("Tracking", frame)
        if cv2.waitKey(1) == ord('q'):
            break
            
    cap.release()
    cv2.destroyAllWindows()

def main():
    model_file = '/home/yangjeongwon/to_students/best.pt'
    if not os.path.exists(model_file):
        print(f"Not found: {model_file}")
        return
    model = YOLO(model_file)
    run_tracking(model)

if __name__ == "__main__":
    main()
