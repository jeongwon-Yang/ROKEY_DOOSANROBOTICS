#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
from ultralytics import YOLO

class BoxDetector(Node):
    def __init__(self):
        super().__init__('box_detector')

        # YOLOv8 모델 로드 (사용자의 경로에 맞게 수정)
        self.model = YOLO('/home/yangjeongwon/Downloads/yolov8s_trained.pt')
        self.get_logger().info("YOLO model loaded.")

        # 이미지 구독 (압축 이미지)
        self.sub_raw = self.create_subscription(
            CompressedImage,
            'image_raw/compressed',   # <-- 퍼블리셔와 동일해야 함
            self.image_callback,
            10
        )

        # 디텍션 결과 이미지 퍼블리셔
        self.pub_detected = self.create_publisher(CompressedImage, 'image_yolo/compressed', 10)

        # 카메라 해상도(픽셀)
        # 만약 카메라가 동적으로 다른 해상도를 제공하면, 아래 값보다는 frame.shape 사용
        self.img_width = 1280
        self.img_height = 720

        # mm/pixel (실험으로 측정한 값. 필요에 따라 조정)
        self.mm_per_pixel = 0.144

        # 화면 중앙선 색 (BGR -> 빨간색)
        self.center_line_color = (0, 0, 255)

    def image_callback(self, msg: CompressedImage):
        # 1) JPEG 디코딩
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is None:
            self.get_logger().error("Failed to decode image.")
            return

        # 2) YOLO 추론
        results = self.model.predict(source=frame, conf=0.25, device='cpu', verbose=False)
        if not results:
            # 디텍션 결과가 없거나 문제가 있는 경우
            self.publish_detected_image(frame)
            return

        # 결과에서 첫 번째 batch만 사용 (YOLOv8의 results는 리스트 형태)
        res = results[0]
        boxes = res.boxes  # 바운딩박스 리스트
        class_names = self.model.names  # index -> 클래스명 매핑

        # 실제 이미지 크기 (frame.shape 기준)
        h, w, _ = frame.shape

        # 3) 화면 중앙에 빨간색 십자선 그리기
        cv2.line(frame, (w // 2, 0), (w // 2, h), self.center_line_color, 2)  # 수직선
        cv2.line(frame, (0, h // 2), (w, h // 2), self.center_line_color, 2)  # 수평선

        # 4) 바운딩박스 처리 (여러 박스를 검출할 수 있으므로 for문)
        for box in boxes:
            # box.xyxy -> [x1, y1, x2, y2]
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            cls_idx = int(box.cls[0])      # 클래스 인덱스
            conf = float(box.conf[0])      # confidence
            class_name = class_names.get(cls_idx, f"{cls_idx}")

            # 바운딩박스 그리기 (초록색)
            cv2.rectangle(
                frame,
                (int(x1), int(y1)),
                (int(x2), int(y2)),
                (0, 255, 0), 2
            )

            # 바운딩박스 중심 좌표(픽셀)
            center_x = (x1 + x2) / 2
            center_y = (y1 + y2) / 2

            # 중심점 찍기 (파란색 점)
            cv2.circle(frame, (int(center_x), int(center_y)), 5, (255, 0, 0), -1)

            # 화면 중앙과의 편차 (dx, dy) - 픽셀 단위
            dx_px = center_x - (w / 2)
            dy_px = center_y - (h / 2)

            # 픽셀 → mm → m 변환
            dx_mm = dx_px * self.mm_per_pixel
            dy_mm = dy_px * self.mm_per_pixel

            dx_m = dx_mm / 1000.0
            dy_m = dy_mm / 1000.0

            # 라벨 텍스트
            # 1) 클래스 이름 + confidence
            label_text_1 = f"{class_name} {conf:.2f}"
            # 2) 원점(화면 중앙)을 (0,0)으로 했을 때의 (dx, dy) [m]
            label_text_2 = f"({dx_m:.3f}m, {dy_m:.3f}m)"

            # 바운딩박스 텍스트를 약간 위쪽에 표시 (예: y1 - 40, y1 - 20)
            text_x = int(x1)
            text_y1 = int(y1) - 40
            text_y2 = int(y1) - 20

            # 화면 밖으로 벗어나지 않도록 최소값 보정
            if text_y1 < 0:
                text_y1 = 0
            if text_y2 < 0:
                text_y2 = 15  # 첫 줄 아래쪽에 맞춰서

            # 첫 줄 (클래스 이름 + confidence)
            cv2.putText(
                frame,
                label_text_1,
                (text_x, text_y1),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2
            )

            # 둘째 줄 (dx, dy in meter)
            cv2.putText(
                frame,
                label_text_2,
                (text_x, text_y2),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 0, 255),  # 붉은 계열로 표시
                2
            )

        # 5) 디텍션 결과 영상 퍼블리시
        self.publish_detected_image(frame)

    def publish_detected_image(self, frame):
        # OpenCV -> JPEG 압축 -> CompressedImage 퍼블리시
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
        success, compressed = cv2.imencode('.jpg', frame, encode_param)
        if not success:
            self.get_logger().error("Failed to encode detected frame.")
            return

        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "yolo"
        msg.format = "jpeg"
        msg.data = compressed.tobytes()

        self.pub_detected.publish(msg)
        # 필요시 로그 출력
        # self.get_logger().info("Published YOLO-detected image.")

def main(args=None):
    rclpy.init(args=args)
    node = BoxDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
