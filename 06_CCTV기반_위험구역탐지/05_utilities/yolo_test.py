import json
import csv
import time
from ultralytics import YOLO
import cv2
import math
import os
import shutil

def run_yolo(model, output_dir):
    # USB 카메라를 사용합니다. 기본 카메라 인덱스가 1로 되어있으나, 환경에 따라 0으로 바꿀 수 있습니다.
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    while True:
        success, frame = cap.read()
        if not success:
            print("카메라 영상을 가져올 수 없습니다.")
            break

        # 실시간 추론 (stream=True 사용 시, 제너레이터 형태로 결과 반환)
        results = model(frame, stream=True)

        object_count = 0
        # 클래스 이름은 모델에 맞게 수정하세요. 예제에서는 두 클래스(Patient, Dummy)를 사용합니다.
        classNames = ['Patient', 'Dummy']

        csv_output = []
        confidences = []
        fontScale = 1
        max_object_count = 0

        for r in results:
            boxes = r.boxes
            for box in boxes:
                # 좌표 추출 및 정수 변환
                x1, y1, x2, y2 = box.xyxy[0]
                x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)

                # 바운딩 박스 그리기
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)

                # 신뢰도 계산
                confidence = math.ceil(box.conf[0] * 100) / 100
                cls = int(box.cls[0])
                confidences.append(confidence)

                # 클래스 이름과 신뢰도 표시
                cv2.putText(frame, f"{classNames[cls]}: {confidence}", (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, fontScale, (255, 0, 0), 2)

                csv_output.append([x1, y1, x2, y2, confidence, cls])
                object_count += 1

        max_object_count = max(max_object_count, object_count)

        # 프레임 상단에 감지된 물체 수 출력
        cv2.putText(frame, f"Objects count: {object_count}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, fontScale, (0, 255, 0), 1)

        # 물체가 감지된 경우 이미지 저장 (파일명은 시간 기반)
        if object_count > 0:
            cv2.imwrite(os.path.join(output_dir, f'output_{int(time.time())}.jpg'), frame)

        # 실시간 화면 출력
        cv2.imshow('Webcam', frame)

        # 'q' 키를 누르면 종료 및 결과 저장
        if cv2.waitKey(1) == ord('q'):
            # CSV 파일 저장
            with open(os.path.join(output_dir, 'output.csv'), 'w', newline='') as file:
                writer = csv.writer(file)
                writer.writerows(csv_output)
            # JSON 파일 저장
            with open(os.path.join(output_dir, 'output.json'), 'w') as file:
                json.dump(csv_output, file)
            # 통계 정보 저장 (최대 감지 물체 수, 평균 신뢰도)
            with open(os.path.join(output_dir, 'statistics.csv'), 'w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(['Max Object Count', 'Average Confidence'])
                writer.writerow([max_object_count, sum(confidences) / len(confidences) if confidences else 0])
            break

    cap.release()
    cv2.destroyAllWindows()

def main():
    # best 모델의 경로를 지정합니다.
    pt_file = '/home/yangjeongwon/to_students/best.pt'
    
    if os.path.exists(pt_file):
        model = YOLO(pt_file)
        output_dir = './output'
        
        # 기존 출력 폴더가 있다면 삭제
        if os.path.exists(output_dir):
            shutil.rmtree(output_dir)
            print(f"The directory {output_dir} has been deleted.")
        
        os.mkdir(output_dir)
        run_yolo(model, output_dir)
    else:
        print(f"Not found: {pt_file}")

if __name__ == "__main__":
    main()
