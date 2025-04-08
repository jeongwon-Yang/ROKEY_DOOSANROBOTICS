import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import math
import os
import threading
import time
import json

def euclidean_distance_rgb(c1, c2):
    """두 RGB 색상 간 유클리드 거리 계산: c1, c2 = (R, G, B)"""
    return math.sqrt((c1[0]-c2[0])**2 + (c1[1]-c2[1])**2 + (c1[2]-c2[2])**2)

class YoloCenterPublisher(Node):
    """
    1) CCTV JSON 알람(result_Yolo_cctv)을 구독하여 (R, G, B) 및 threshold 업데이트  
    2) 카메라 + YOLO(TensorRT, task='detect') 및 내부 Tracker 기능 활성화 (bytetrack.yaml 사용)  
    3) 기본적으로 ByteTrack을 이용해 움직이는 객체(Tracker 대상)를 추적하고,
       만약 화면에서 대상이 사라지거나 누락되면, 이전에 받은 색상 값(cctv_rgb)을 기준으로 후보 객체를 재탐색하여
       다시 해당 객체의 Tracking을 재개함.  
    4) [center_x, center_y, height]를 bounding_box/center 토픽으로 publish  
    5) 처리된 영상을 압축 이미지 'Img_Yolo_AMR' 토픽으로 publish (GUI 표시용)
    
    ※ 테스트 목적을 위해, GUI Tracking 입력이 없더라도 기본 색상값으로 Tracking이 실행되도록 구성함.
    """
    def __init__(self):
        super().__init__('yolo_center_publisher')

        # (1) CCTV JSON 구독: 색상값 업데이트
        self.json_sub = self.create_subscription(
            String,
            'result_Yolo_cctv',
            self.json_callback,
            10
        )
        self.cctv_rgb = None
        self.color_dist_threshold = 60.0  # 색상 거리 임계값

        # 테스트 목적: GUI Tracking 입력이 없더라도 기본값을 사용하도록 시뮬레이션 플래그 활성화
        self.simulate_gui_tracking = True  
        self.default_cctv_rgb = (150, 100, 100)

        # (2) bounding_box/center 및 이미지 publish
        self.center_pub = self.create_publisher(Float32MultiArray, 'bounding_box/center', 10)
        self.image_pub = self.create_publisher(CompressedImage, 'Img_Yolo_AMR', 10)
        self.bridge = CvBridge()

        # TensorRT 엔진 파일 경로 확인
        engine_file = '/home/rokey2/rokey2_E1_ws/src/intelligence2/intelligence2/batch_16_epoch_20_1000.engine'
        if not os.path.exists(engine_file):
            self.get_logger().error(f"Engine file not found: {engine_file}")
            rclpy.shutdown()

        # YOLO 모델 로드 (생성자에 tracker 인자는 제거)
        self.model = YOLO(engine_file, task='detect')

        # 카메라 초기화 (필요 시 인덱스 변경)
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        # 클래스 이름 (예: 단일 클래스 "Patient")
        self.class_names = ["Patient"]

        # 현재 추적 중인 객체의 ID (Tracker를 통해 할당된)
        self.current_track_id = None

        # 별도 스레드에서 추적 루프 실행
        threading.Thread(target=self.tracking_loop, daemon=True).start()

    def json_callback(self, msg):
        """
        CCTV JSON 메시지: { "rgb_values": {"R":..., "G":..., "B":...}, ... }
        """
        try:
            data = json.loads(msg.data)
            self.get_logger().info(f"CCTV JSON: {data}")
            if "rgb_values" in data:
                r_val = data["rgb_values"].get("R", 150)
                g_val = data["rgb_values"].get("G", 100)
                b_val = data["rgb_values"].get("B", 100)
                self.cctv_rgb = (r_val, g_val, b_val)
                self.get_logger().info(f"[ALARM] cctv_rgb={self.cctv_rgb}")
        except Exception as e:
            self.get_logger().error(f"JSON parse error: {e}")

    def tracking_loop(self):
        # 경로에 있는 bytetrack.yaml 파일을 tracker 설정으로 사용
        tracker_config = '/home/rokey2/rokey2_E1_ws/src/intelligence2/intelligence2/bytetrack.yaml'
        while rclpy.ok():
            try:
                ret, frame = self.cap.read()
                if not ret:
                    self.get_logger().warn("Failed to capture frame from camera")
                    time.sleep(0.1)
                    continue

                # 테스트 시, 만약 GUI Tracking 입력이 없다면 기본값 할당
                if self.simulate_gui_tracking and self.cctv_rgb is None:
                    self.cctv_rgb = self.default_cctv_rgb

                best_distance = float('inf')
                target_box = None
                candidate_track_id = None
                found_target = False

                # ByteTrack을 사용한 추론: tracker 인자는 track 메서드에 전달
                results = self.model.track(source=frame, show=False, tracker=tracker_config)
                for result in results:
                    for box in result.boxes:
                        # 바운딩 박스 좌표 및 confidence 값
                        x1, y1, x2, y2 = map(int, box.xyxy[0])
                        conf = float(box.conf[0]) if box.conf else 0.0
                        if conf < 0.8:
                            continue
                        track_id = int(box.id[0]) if box.id is not None else -1

                        # ROI 영역 및 평균 색상 계산 (BGR → RGB)
                        roi = frame[y1:y2, x1:x2]
                        if roi.size == 0:
                            continue
                        avg_bgr = cv2.mean(roi)[:3]
                        avg_rgb = (int(avg_bgr[2]), int(avg_bgr[1]), int(avg_bgr[0]))

                        # (1) 이미 추적 중인 객체(ID가 일치)가 있으면 우선 선택
                        if self.current_track_id is not None and track_id == self.current_track_id:
                            target_box = (x1, y1, x2, y2)
                            candidate_track_id = track_id
                            found_target = True
                            break  # 현재 프레임에서 대상 확인
                        else:
                            # (2) 추적 중인 객체가 없거나 ID가 일치하지 않으면 색상 기반 후보 선택
                            if self.cctv_rgb is not None:
                                distance = euclidean_distance_rgb(avg_rgb, self.cctv_rgb)
                                if distance < self.color_dist_threshold and distance < best_distance:
                                    best_distance = distance
                                    candidate_track_id = track_id
                                    target_box = (x1, y1, x2, y2)
                                    found_target = True
                        # 라벨 표시: Tracking ID, 색상 거리, confidence
                        label_text = f"ID:{track_id} d={best_distance:.1f} c={conf:.2f}"
                        box_color = (0, 255, 0)
                        if self.cctv_rgb is not None:
                            distance = euclidean_distance_rgb(avg_rgb, self.cctv_rgb)
                            if distance < self.color_dist_threshold:
                                box_color = (0, 0, 255)
                        cv2.rectangle(frame, (x1, y1), (x2, y2), box_color, 2)
                        cv2.putText(frame, label_text, (x1, y1 - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, box_color, 2)
                    if found_target:
                        break  # 한 프레임 내에서 대상이 확인되면 추가 탐색 중단

                # 대상 객체가 식별되었으면 current_track_id 업데이트 (tracking 재개)
                if target_box is not None:
                    self.current_track_id = candidate_track_id
                    x1, y1, x2, y2 = target_box
                    center_x = (x1 + x2) / 2.0
                    center_y = (y1 + y2) / 2.0
                    height   = (y2 - y1)
                    msg = Float32MultiArray()
                    msg.data = [float(center_x), float(center_y), float(height)]
                    self.get_logger().info(f"[TRACK] center=({center_x:.1f},{center_y:.1f}), h={height:.1f}, ID={self.current_track_id}")
                else:
                    self.current_track_id = None
                    msg = Float32MultiArray()
                    msg.data = [0.0, 0.0, 0.0]
                    self.get_logger().info("[TRACK] No target matched, zeros.")

                self.center_pub.publish(msg)
                compressed_img = self.bridge.cv2_to_compressed_imgmsg(frame, dst_format="jpg")
                self.image_pub.publish(compressed_img)
                time.sleep(0.05)
            except Exception as e:
                self.get_logger().error(f"Error in tracking loop: {e}")
                time.sleep(0.1)
                continue

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = YoloCenterPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down YoloCenterPublisher node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
