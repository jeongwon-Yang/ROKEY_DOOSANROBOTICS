#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
import cv2
import numpy as np
from ultralytics import YOLO

class BoxDetector(Node):
    def __init__(self):
        super().__init__('box_detector')

        # YOLOv8 모델 로드
        self.model = YOLO('/home/.../yolov8s_trained.pt')
        self.get_logger().info("YOLO model loaded.")

        # 압축 이미지 구독
        self.sub_raw = self.create_subscription(
            CompressedImage,
            'image_raw/compressed',  # <- 실제 카메라 토픽명 확인
            self.image_callback,
            10
        )

        # 디텍션 결과 이미지 퍼블리셔
        self.pub_detected = self.create_publisher(CompressedImage, 'image_yolo/compressed', 10)

        # **파이썬 리스트(문자열) 퍼블리시할 토픽
        self.pub_boxinfo = self.create_publisher(String, 'yolo/detected_info', 10)

        # 파라미터
        self.mm_per_pixel = 0.144  # 실험값

    def image_callback(self, msg: CompressedImage):
        # 1) 이미지 디코딩
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is None:
            self.get_logger().error("Failed to decode image.")
            return

        # 2) YOLO 추론
        results = self.model.predict(source=frame, conf=0.25, device='cpu', verbose=False)
        if not results:
            # 인식 결과가 없으면 그냥 이미지만 퍼블리시
            self.publish_detected_image(frame)
            return

        res = results[0]
        boxes = res.boxes
        class_names = self.model.names

        h, w, _ = frame.shape
        # 중앙선 표시 (옵션)
        cv2.line(frame, (w // 2, 0), (w // 2, h), (0, 0, 255), 2)
        cv2.line(frame, (0, h // 2), (w, h // 2), (0, 0, 255), 2)

        # 이 리스트에 [class, x, y] 형태로 저장
        detected_list = []

        # 4) 바운딩박스
        for i, box in enumerate(boxes):
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            cls_idx = int(box.cls[0])
            conf = float(box.conf[0])
            class_name = class_names.get(cls_idx, str(cls_idx))

            # 중심점 계산
            center_x = (x1 + x2) / 2
            center_y = (y1 + y2) / 2

            dx_px = center_x - (w / 2)
            dy_px = center_y - (h / 2)
            dx_m = dx_px * self.mm_per_pixel / 1000.0
            dy_m = dy_px * self.mm_per_pixel / 1000.0

            # 바운딩박스 표시
            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
            cv2.circle(frame, (int(center_x), int(center_y)), 5, (255, 0, 0), -1)

            # 텍스트
            label_text_1 = f"{class_name} {conf:.2f}"
            label_text_2 = f"({dx_m:.3f}, {dy_m:.3f})"
            cv2.putText(frame, label_text_1, (int(x1), int(y1)-25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
            cv2.putText(frame, label_text_2, (int(x1), int(y1)-5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,255), 2)

            # 오케스트라 노드와 맞추기 위해 [class, x, y] 형태로 저장
            # 오케스트라 노드는 "data_list[0][1], data_list[0][2]" => x, y
            # 즉 detected_list[i] = [class_name, x_m, y_m]
            detected_list.append([class_name, dx_m, dy_m])

        # (선택) 정렬이 필요하다면 여기서 할 수도 있습니다.
        # detected_list.sort(key=lambda item: (item[1], item[2]))  # 예: x_m, y_m 순서

        # 5) 퍼블리시할 메시지 작성 (파이썬 리스트를 문자열로)
        string_msg = String()
        # 오케스트라 노드에서 ast.literal_eval()로 파싱 예정
        string_msg.data = str(detected_list)

        # 6) 토픽 퍼블리시
        self.pub_boxinfo.publish(string_msg)
        self.publish_detected_image(frame)

    def publish_detected_image(self, frame):
        _, compressed = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.format = "jpeg"
        msg.data = compressed.tobytes()
        self.pub_detected.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = BoxDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

