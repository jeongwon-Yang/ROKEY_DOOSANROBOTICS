import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import json
import datetime
import os
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from ultralytics import YOLO
from cv_bridge import CvBridge
# 위험 구역 폴리라인 저장 변수
polygon_points = np.array([(239, 125), (556, 114), (587, 335), (192, 361)], np.int32)
setup_complete = True  # 이미 폴리라인이 설정되어 있으므로 True로 변경
warning_mode = False  # 경고 상태
# R 값 임계값
R_THRESHOLD = 150
# 고정된 목표 좌표 값 (실수형)
goal_x = 3.0
goal_y = 12.5
# JSON 로그 파일 경로
# json_log_path = "/home/songhyun/alert_log.json"
# 객체 추적 딕셔너리
object_tracking = {}
class YoloCCTVNode(Node):
    def __init__(self):
        super().__init__("yolo_cctv_node")
        print(":로켓: YOLO CCTV 노드 초기화 완료")
        # 퍼블리셔 정의
        self.image_pub = self.create_publisher(CompressedImage, "Img_Yolo_cctv", 10)
        self.json_pub = self.create_publisher(String, "result_Yolo_cctv", 10)
        # CvBridge 객체 생성 (GUI 연동을 위해 필요)
        self.bridge = CvBridge()
        # 카메라 열기
        self.cap = cv2.VideoCapture(4)
        if not self.cap.isOpened():
            self.get_logger().error(":경광등: 카메라를 열 수 없습니다. 종료합니다.")
            return
        self.model = YOLO('/home/yangjeongwon/to_students/best.pt')
        # 타이머 설정 (0.1초마다 실행)
        self.timer = self.create_timer(0.1, self.process_frame)
    def process_frame(self):
        global setup_complete, polygon_points
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error(":경고: 프레임을 읽을 수 없습니다.")
            return
        detected_in_danger_zone = False
        frame_copy = frame.copy()
        cv2.polylines(frame_copy, [polygon_points], isClosed=True, color=(255, 0, 0), thickness=2)
        # YOLO 감지 실행
        results = self.model(frame, verbose=False)
        # 감지된 객체 확인
        for result in results:
            for box in result.boxes:
                coords = box.xyxy[0].tolist()
                x1, y1, x2, y2 = map(int, coords)
                center_x, center_y = (x1 + x2) // 2, (y1 + y2) // 2
                conf = float(box.conf.item()) if box.conf is not None else 0.0
                if conf < 0.70:
                    continue
                # 객체 클래스명 가져오기
                class_id = int(box.cls.item())
                object_name = self.model.names[class_id] if class_id in self.model.names else "Unknown"
                # 바운딩 박스 & 라벨 표시
                cv2.rectangle(frame_copy, (x1, y1), (x2, y2), (0, 255, 0), 2)
                label = f"{object_name} {conf:.2f}"
                cv2.putText(frame_copy, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                # 객체가 폴리라인 안에 있는지 확인
                if setup_complete and len(polygon_points) > 2:
                    pts_polygon = np.array(polygon_points, np.int32)
                    inside = cv2.pointPolygonTest(pts_polygon, (center_x, center_y), False)
                    if inside >= 0:
                        obj_roi = frame[y1:y2, x1:x2]
                        if obj_roi.size == 0:
                            self.get_logger().warn(":경고: ROI 크기가 0. 객체를 건너뜁니다.")
                            continue
                        avg_color = cv2.mean(obj_roi)[:3]
                        b_value, g_value, r_value = int(avg_color[0]),int(avg_color[1]), int(avg_color[2])
                        self.get_logger().info(f":압정: 객체 감지됨: {object_name}, R: {r_value}, G: {g_value}, B: {b_value}, 위치: {center_x}, {center_y}")
                        # R 값이 150 이상이면 경고 + JSON 저장 (한 번만)
                        if r_value >= R_THRESHOLD:
                            detected_in_danger_zone = True
                            timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                            if object_name not in object_tracking:
                                object_tracking[object_name] = True
                                alert_data = {
                                    "timestamp": timestamp,
                                    "object": object_name,
                                    "confidence": round(conf, 2),
                                    "rgb_values": {"R": r_value, "G": g_value, "B": b_value},
                                    "goal": {"x": float(goal_x), "y": float(goal_y)}
                                }
                                self.publish_json(alert_data)
        # 객체가 폴리라인 안에 있고 R 값이 150 이상이면 경고 효과 표시
        if detected_in_danger_zone:
            overlay = frame_copy.copy()
            cv2.rectangle(overlay, (0, 0), (frame.shape[1], frame.shape[0]), (0, 0, 255), -1)
            cv2.addWeighted(overlay, 0.3, frame_copy, 0.7, 0, frame_copy)
            cv2.putText(frame_copy, "DANGER! HIGH R VALUE DETECTED!", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 3, cv2.LINE_AA)
        # YOLO 영상 퍼블리시 (GUI가 수신할 수 있도록)
        self.publish_image(frame_copy)
    def publish_image(self, frame):
        """YOLO 감지 영상 퍼블리시 (CompressedImage)"""
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', frame)[1]).tobytes()
        self.image_pub.publish(msg)
    def publish_json(self, data):
        """객체 감지 결과 JSON 퍼블리시"""
        msg = String()
        msg.data = json.dumps(data)
        self.json_pub.publish(msg)
        self.get_logger().info(f":확성기: JSON 퍼블리시: {data}")
        # try:
        #     with open(json_log_path, "a") as json_file:
        #         json.dump(data, json_file)
        #         json_file.write("\n")  # 개행 추가
        #     self.get_logger().info(f":흰색_확인_표시: JSON 파일 저장 완료: {json_log_path}")
        # except Exception as e:
        #     self.get_logger().error(f":x: JSON 파일 저장 실패: {e}")
def main(args=None):
    rclpy.init(args=args)
    node = YoloCCTVNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
if __name__ == "__main__":
    main()