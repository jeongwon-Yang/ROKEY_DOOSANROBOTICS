#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
import cv2
import numpy as np
import json
from ultralytics import YOLO

class BoxDetector(Node):
    def __init__(self):
        super().__init__('box_detector')

        # YOLOv8 모델 로드
        self.model = YOLO('/home/yangjeongwon/Downloads/yolov8s_trained.pt')
        self.get_logger().info("YOLO model loaded.")

        # 이미지 구독 (압축 이미지)
        self.sub_raw = self.create_subscription(
            CompressedImage,
            'image_raw/compressed',
            self.image_callback,
            10
        )

        # 디텍션 결과 이미지 퍼블리셔
        self.pub_detected = self.create_publisher(CompressedImage, 'image_yolo/compressed', 10)

        # **추가**: 바운딩박스 좌표를 JSON으로 퍼블리시할 토픽
        self.pub_boxinfo = self.create_publisher(String, 'yolo/detected_info', 10)

        # 카메라 해상도(픽셀)
        self.img_width = 1280
        self.img_height = 720

        # mm/pixel (실험으로 측정한 값)
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
            self.publish_detected_image(frame)
            return

        res = results[0]
        boxes = res.boxes
        class_names = self.model.names

        # 실제 이미지 크기
        h, w, _ = frame.shape

        # 3) 화면 중앙에 빨간색 십자선
        cv2.line(frame, (w // 2, 0), (w // 2, h), self.center_line_color, 2)
        cv2.line(frame, (0, h // 2), (w, h // 2), self.center_line_color, 2)

        # 바운딩박스 정보를 저장할 리스트
        boxes_data = []

        # 4) 바운딩박스 처리
        for i, box in enumerate(boxes):
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            cls_idx = int(box.cls[0])
            conf = float(box.conf[0])
            class_name = class_names.get(cls_idx, f"{cls_idx}")

            # 바운딩박스 그리기
            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)

            # 1) 클래스 & confidence
            label_text_1 = f"{class_name} {conf:.2f}"

            # 바운딩박스 중심
            center_x = (x1 + x2) / 2
            center_y = (y1 + y2) / 2

            # 화면 중앙 대비 픽셀 편차
            dx_px = center_x - (w / 2)
            dy_px = center_y - (h / 2)

            # 픽셀 → mm → m
            dx_m = dx_px * self.mm_per_pixel / 1000.0
            dy_m = dy_px * self.mm_per_pixel / 1000.0

            # 2) (dx_m, dy_m)을 표시
            label_text_2 = f"({dx_m:.3f}m, {dy_m:.3f}m)"

            # 바운딩박스 라벨 위치
            text_x = int(x1)
            text_y1 = int(y1) - 40
            text_y2 = int(y1) - 20
            if text_y1 < 0: text_y1 = 0
            if text_y2 < 0: text_y2 = 15

            cv2.putText(frame, label_text_1, (text_x, text_y1),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(frame, label_text_2, (text_x, text_y2),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

            # 중심점 찍기 (파란색)
            cv2.circle(frame, (int(center_x), int(center_y)), 5, (255, 0, 0), -1)

            # boxes_data에 추가할 dict
            box_info = {
                "index": i,                 # 박스 인덱스
                "class": class_name,        # 분류명
                "confidence": conf,
                "x_m": dx_m,               # 화면 중앙 (0,0) 기준 m 단위
                "y_m": dy_m,
                "x_px": int(center_x),      # 디버깅용 픽셀 좌표
                "y_px": int(center_y)
            }
            boxes_data.append(box_info)

        # (선택) 5) 왼쪽 위(Top-Left)부터 차례대로 정렬하려면:
        #  - 화면에서 "top-left" = y가 작을수록 "위", x가 작을수록 "왼쪽"
        #  - 따라서 (y_px, x_px)를 기준으로 정렬
        #  - 필요한 경우 주석 해제
        boxes_data.sort(key=lambda b: (b["y_px"], b["x_px"]))

        # 6) JSON으로 변환하여 퍼블리시
        #    (원하는 정보만 담아도 됩니다)
        data_dict = {"boxes": boxes_data}
        json_msg = String()
        json_msg.data = json.dumps(data_dict, ensure_ascii=False)
        self.pub_boxinfo.publish(json_msg)

        # 7) 디텍션 결과 이미지 퍼블리시
        self.publish_detected_image(frame)

    def publish_detected_image(self, frame):
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
        # self.get_logger().info("Published YOLO-detected image.")

def main(args=None):
    rclpy.init(args=args)
    node = BoxDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
