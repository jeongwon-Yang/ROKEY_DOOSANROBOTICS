#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        
        # 퍼블리셔 생성
        self.publisher_ = self.create_publisher(CompressedImage, 'image_raw/compressed', 10)
        
        # 타이머 (0.1초마다 -> 10fps)
        self.timer = self.create_timer(0.1, self.publish_image)

        # 카메라 오픈 (예: /dev/video0)
        self.cap = cv2.VideoCapture(2, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            self.get_logger().error("Cannot open camera (/dev/video2).")
            return

        # 카메라 설정 (원하면 해상도 등 변경)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M','J','P','G'))
        self.cap.set(cv2.CAP_PROP_FPS, 25)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

        w = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        h = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        self.get_logger().info(f"Camera opened: {w}x{h}")

    def publish_image(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Failed to grab frame.")
            return

        # JPEG 압축
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
        success, compressed = cv2.imencode('.jpg', frame, encode_param)
        if not success:
            self.get_logger().error("Failed to encode frame.")
            return

        # 메시지 생성
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera"
        msg.format = "jpeg"
        msg.data = compressed.tobytes()

        # 퍼블리시
        self.publisher_.publish(msg)
        self.get_logger().info("Publishing compressed image...")

def main(args=None):
    rclpy.init(args=args)
    node = ImagePublisher()
    rclpy.spin(node)
    node.cap.release()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

