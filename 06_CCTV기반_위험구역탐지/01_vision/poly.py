import cv2
import numpy as np
from ultralytics import YOLO
# :번쩍: YOLO 모델 로드 (사용자 모델 사용)
model = YOLO('batch_16_epoch_20_1000.pt')
# :다트: 다각형(폴리라인) 저장 변수
polygon_points = []
drawing = False         # 현재 폴리라인 그리는 중 여부
warning_mode = False    # 침입 감지 상태 (ON/OFF)
setup_complete = False  # 위험 구역 설정 완료 여부
# :로켓: 웹캠 열기 (USB캠 인덱스는 환경에 따라 조정)
cap = cv2.VideoCapture(4)
# :압정: 마우스 이벤트 콜백 함수 (폴리라인 설정)
def draw_polygon(event, x, y, flags, param):
    global polygon_points, drawing, setup_complete
    if setup_complete:
        return  # 설정 완료 후 추가 설정 불가
    if event == cv2.EVENT_LBUTTONDOWN:
        polygon_points.append((x, y))  # 클릭한 좌표 추가
        print(f"포인트 추가: {(x, y)}")
    elif event == cv2.EVENT_RBUTTONDOWN and len(polygon_points) > 2:
        drawing = True  # 다각형 설정 완료
        setup_complete = True
        print("위험 구역 설정 완료!")
# :예술: OpenCV 윈도우 생성 및 마우스 이벤트 연결
cv2.namedWindow("YOLO Intrusion Detection")
cv2.setMouseCallback("YOLO Intrusion Detection", draw_polygon)
print("영상을 클릭하여 위험 구역(폴리라인)을 설정하세요. 완료되면 'f' 키를 누르세요.")
while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        continue
    # :압정: 위험 구역 설정 표시
    if len(polygon_points) > 1:
        pts = np.array(polygon_points, np.int32).reshape((-1, 1, 2))
        cv2.polylines(frame, [pts], isClosed=True, color=(0, 255, 0), thickness=2)
    for point in polygon_points:
        cv2.circle(frame, point, 5, (0, 0, 255), -1)  # 포인트 시각화
    # 위험 구역 설정 완료 후 YOLO 감지 시작
    if setup_complete:
        results = model(frame, verbose=False)
        detected = False  # 침입 여부 상태 변수
        pts_polygon = np.array(polygon_points, np.int32)
        # :작은_파란색_다이아몬드: 감지된 객체 확인
        for result in results:
            for box in result.boxes:
                # 바운딩 박스 좌표 추출
                coords = box.xyxy[0].tolist()
                x1, y1, x2, y2 = map(int, coords)
                center_x, center_y = (x1 + x2) // 2, (y1 + y2) // 2
                # confidence 값 추출 후 0.7 미만은 무시
                conf = float(box.conf[0]) if box.conf is not None else 0.0
                if conf < 0.7:
                    continue
                # 클래스 이름은 "Patient"로 고정 (confidence score 포함)
                label = f"Patient {conf:.2f}"
                # :큰_빨간색_네모: 바운딩 박스 그리기 및 텍스트 출력
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
                cv2.putText(frame, label, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, (0, 0, 255), 2, cv2.LINE_AA)
                # :작은_파란색_다이아몬드: 객체 중심이 위험 구역(폴리곤) 내부에 있는지 확인
                inside = cv2.pointPolygonTest(pts_polygon, (center_x, center_y), False)
                if inside >= 0:
                    detected = True
                    warning_mode = True  # 경고 효과 ON
                    # :큰_파란색_원: 객체의 평균 색상(BGR → RGB 변환 후 터미널 출력)
                    obj_roi = frame[y1:y2, x1:x2]
                    if obj_roi.size != 0:
                        avg_color = cv2.mean(obj_roi)[:3]
                        avg_color_rgb = (int(avg_color[2]), int(avg_color[1]), int(avg_color[0]))
                        print(f":경고: 위험지역 침입 감지 - 평균 색상 (RGB): {avg_color_rgb}")
                    # 한 객체만 경고 내도 충분하면 break로 나갈 수 있음
                    break
        # :경광등: 침입 감지 시 화면 경고 효과 (빨간 틴트 및 경고 메시지)
        if warning_mode:
            overlay = frame.copy()
            alpha = 0.6  # 투명도
            color = (0, 0, 255)  # 빨간색
            cv2.rectangle(overlay, (0, 0), (frame.shape[1], frame.shape[0]), color, -1)
            cv2.addWeighted(overlay, alpha, frame, 1 - alpha, 0, frame)
            cv2.putText(frame, "WARNING! INTRUSION DETECTED!", (50, 50), cv2.FONT_HERSHEY_SIMPLEX,
                        1, (255, 255, 255), 3, cv2.LINE_AA)
        # :출입금지_기호: 객체가 없거나 위험 구역을 벗어나면 경고 해제
        if not detected:
            warning_mode = False
    # :데스크톱_컴퓨터: 화면 출력
    cv2.imshow("YOLO Intrusion Detection", frame)
    # :중지를_나타내는_검은_정사각형: 키 입력 처리 ('f' 키로 위험 구역 설정 완료, ESC 키로 종료)
    key = cv2.waitKey(1) & 0xFF
    if key == ord('f') and len(polygon_points) >= 3:
        print("위험 구역이 설정되었습니다.")
        setup_complete = True
    if key == 27:  # ESC 키
        break
# 정리
cap.release()
cv2.destroyAllWindows()