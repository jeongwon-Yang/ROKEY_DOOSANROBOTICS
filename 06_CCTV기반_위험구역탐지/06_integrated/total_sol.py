import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import Float32MultiArray, String
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, Quaternion

from cv_bridge import CvBridge
from ultralytics import YOLO

import cv2
import numpy as np
import math
import os
import threading
import time
import json


########################################
# 1) YOLO Node: YoloCenterPublisher
########################################

def euclidean_distance_rgb(c1, c2):
    """두 RGB 색상(R, G, B) 간 유클리드 거리 계산"""
    return math.sqrt((c1[0]-c2[0])**2 + (c1[1]-c2[1])**2 + (c1[2]-c2[2])**2)

class YoloCenterPublisher(Node):
    """
    - CCTV JSON(result_Yolo_cctv) 구독 → (R, G, B)
    - YOLO 추론 + tracking
    - 대상 바운딩 박스 중심 + 높이를 bounding_box/center로 발행
    - 바운딩 박스 움직임 궤적(trajectory) 선으로 표시
    - CompressedImage(Img_Yolo_AMR)로 publish
    """

    def __init__(self):
        super().__init__('yolo_center_publisher')

        # JSON 구독 (CCTV 알람)
        self.json_sub = self.create_subscription(String, 'result_Yolo_cctv', self.json_callback, 10)
        self.cctv_rgb = None
        self.color_dist_threshold = 60.0

        # bounding_box/center, Img_Yolo_AMR 발행
        self.center_pub = self.create_publisher(Float32MultiArray, 'bounding_box/center', 10)
        self.image_pub  = self.create_publisher(CompressedImage, 'Img_Yolo_AMR', 10)
        self.bridge = CvBridge()

        # TensorRT 엔진 로드, task='detect'로 경고 제거
        engine_file = '/home/rokey2/rokey2_E1_ws/src/intelligence2/intelligence2/batch_16_epoch_20_1000.engine'
        if not os.path.exists(engine_file):
            self.get_logger().error(f"Engine file not found: {engine_file}")
            rclpy.shutdown()
        self.model = YOLO(engine_file, task='detect')

        # 카메라 초기화
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        # 클래스 이름(예시)
        self.class_names = ["Patient"]

        # 바운딩박스 중심 궤적
        self.trajectory_points = []
        self.max_points = 100  # 저장할 최대 점 개수

        # 별도 스레드로 loop
        threading.Thread(target=self.tracking_loop, daemon=True).start()

    def json_callback(self, msg):
        """
        CCTV JSON (예시)
        {
          "timestamp": "...",
          "rgb_values": {"R":180, "G":120, "B":100},
          "object":"Patient"
        }
        """
        try:
            data = json.loads(msg.data)
            self.get_logger().info(f"Received CCTV JSON: {data}")
            if "rgb_values" in data:
                r_val = data["rgb_values"].get("R", 150)
                g_val = data["rgb_values"].get("G", 100)
                b_val = data["rgb_values"].get("B", 100)
                self.cctv_rgb = (r_val, g_val, b_val)
                self.get_logger().info(f"[ALARM] cctv_rgb={self.cctv_rgb}")
        except Exception as e:
            self.get_logger().error(f"JSON parse error: {e}")

    def tracking_loop(self):
        """
        - model.track() 사용하여 YOLO tracking 기능 활용
        - cctv_rgb와 색상 거리가 가까운 객체 -> 빨간 박스
        - 가장 유사한 객체 -> bounding_box/center publish
        - 바운딩박스 중심 trajectory 선 표시
        """
        while rclpy.ok():
            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().warn("Failed to capture frame from camera")
                time.sleep(0.1)
                continue
            
            # YOLO Tracking (persist=True)
            results = self.model.track(source=frame, persist=True)
            
            best_distance = float('inf')
            target_box = None

            if len(results) > 0:
                boxes = results[0].boxes
                for box in boxes:
                    track_id = box.id.cpu().item() if box.id is not None else None
                    coords = box.xyxy[0].tolist()
                    x1, y1, x2, y2 = map(int, coords)
                    conf = float(box.conf[0]) if box.conf is not None else 0.0
                    # ROI 평균색
                    roi = frame[y1:y2, x1:x2]
                    if roi.size == 0:
                        continue
                    avg_bgr = cv2.mean(roi)[:3]
                    avg_rgb = (int(avg_bgr[2]), int(avg_bgr[1]), int(avg_bgr[0]))

                    box_color = (0,255,0)
                    distance = float('inf')
                    if self.cctv_rgb is not None:
                        distance = euclidean_distance_rgb(avg_rgb, self.cctv_rgb)
                        if distance < self.color_dist_threshold:
                            box_color = (0,0,255)  # 빨간 박스
                            if distance < best_distance:
                                best_distance = distance
                                target_box = (x1,y1,x2,y2)

                    label_text = f"id:{track_id} c:{conf:.2f}"
                    cv2.rectangle(frame, (x1,y1), (x2,y2), box_color, 2)
                    cv2.putText(frame, label_text, (x1,y1-10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, box_color,2)

            # target_box → [cx,cy,height] publish
            msg = Float32MultiArray()
            if target_box is not None and best_distance!=float('inf'):
                x1,y1,x2,y2 = target_box
                cx = (x1 + x2)/2.0
                cy = (y1 + y2)/2.0
                h  = (y2 - y1)
                msg.data = [cx, cy, float(h)]
                self.get_logger().info(f"[TRACK] center=({cx:.1f},{cy:.1f}), height={h:.1f}")
                # trajectory
                self.trajectory_points.append((int(cx), int(cy)))
                if len(self.trajectory_points)>self.max_points:
                    self.trajectory_points.pop(0)
            else:
                msg.data = [0.0,0.0,0.0]
                self.get_logger().info("[TRACK] No target => zeros.")
            
            self.center_pub.publish(msg)

            # 궤적(trajectory) 그리기 (빨간 선)
            for i in range(1, len(self.trajectory_points)):
                pt1 = self.trajectory_points[i-1]
                pt2 = self.trajectory_points[i]
                cv2.line(frame, pt1, pt2, (0,0,255), 2)

            # 압축 이미지 publish
            comp_img = self.bridge.cv2_to_compressed_imgmsg(frame, dst_format="jpg")
            self.image_pub.publish(comp_img)
            
            time.sleep(0.05)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()


########################################
# 2) 로봇 이동/추적 노드: MoveNode
########################################

class MoveNode(Node):
    """
    - 'robot_status' 수신 → 상태 변화
    - 'bounding_box/center' → (cx, cy, height) 기반 사분면 로직
    - 위험 경보 시 Zone6로 이동, 이동 중 특정 바운딩박스 발견 시 추적
    - 로그로 GUI에 상태 알림
    """

    def __init__(self):
        super().__init__('move_node')
        # GUI로부터 로봇 상태(Robot_status) 수신
        self.robot_status_sub = self.create_subscription(String, 'robot_status', self.robot_status_callback, 10)
        # YOLO에서 추적 바운딩박스 결과 수신
        self.create_subscription(Float32MultiArray, 'bounding_box/center', self.bbox_callback, 10)

        # cmd_vel 퍼블리셔
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # 상태 변수
        self.robot_status = "READY"
        self.goal_sent = False

        # 이미지 해상도 가정
        self.image_width  = 640
        self.image_height = 480
        self.threshold_x = 40
        self.threshold_y = 40

        # 속도 파라미터
        self.angular_speed = 0.15
        self.angular_speed_search = 0.20
        self.linear_speed_far   = 0.10
        self.linear_speed_close = 0.05
        self.height_far   = 140
        self.height_close = 180

        # 타이머
        self.create_timer(1.0, self.timer_callback)

    def robot_status_callback(self, msg):
        self.robot_status = msg.data
        self.get_logger().info(f"[MOVE] Robot_status updated: {self.robot_status}")
        # 상태 전환 시 goal_sent 등 초기화 가능
        if self.robot_status in ["Detected", "Quest"]:
            self.goal_sent = False

    def bbox_callback(self, msg):
        """
        bounding_box/center → [center_x, center_y, height]
        """
        data = msg.data
        if len(data)<3:
            self.get_logger().warn("Need [cx, cy, height], got less.")
            return
        
        cx, cy, height = data

        # 로그
        self.get_logger().info(f"[MOVE] Bbox center=({cx:.1f},{cy:.1f}), h={height:.1f}, status={self.robot_status}")

        # 만약 로봇이 "Tracking" 상태 or "Detected" 상태에서 이 bbox를 보고 추적을 시작할 수도 있음
        # 예) 아래에서는 "tracking" 상태라고 가정 시, quadrant 로직으로 이동
        # OR "Detected" 상태여도 -> if we want to override ?

        if self.robot_status == "Tracking":
            # quadrant approach
            if cx==0.0 and cy==0.0:
                # no target => rotate
                self.publish_cmd_vel(0.0, self.angular_speed_search)
                self.get_logger().info("[MOVE] No target => rotating in place")
            else:
                # 사분면
                mid_x = self.image_width/2.0
                mid_y = self.image_height/2.0
                error_x = cx - mid_x
                error_y = cy - mid_y

                is_left = (error_x < -self.threshold_x)
                is_right= (error_x >  +self.threshold_x)
                # is_top   = (error_y < -self.threshold_y)
                # is_bottom= (error_y >  +self.threshold_y)

                # 회전
                angular_z = 0.0
                if is_left:
                    angular_z = +self.angular_speed
                elif is_right:
                    angular_z = -self.angular_speed

                # 거리
                linear_x = 0.0
                if height<self.height_far:
                    linear_x = self.linear_speed_far
                elif height>self.height_close:
                    linear_x = -self.linear_speed_close
                else:
                    linear_x = self.linear_speed_close

                self.publish_cmd_vel(linear_x, angular_z)
        else:
            # 다른 상태일 때는 무시 or 로봇이 목표 지점 이동 중
            pass

    def timer_callback(self):
        """
        상태에 따라 동작:
        - Detected → 이동 명령
        - Quest → 순차 경로
        - Tracking → bbox_callback에서 제어
        - READY → 대기
        """
        if self.robot_status=="Detected":
            if not self.goal_sent:
                # Zone6 위치로 가정
                self.get_logger().info("[MOVE] Danger! Going to Zone6. (demo: just logging, no nav2)") 
                # 실제 nav2 goal 전송 코드 ...
                self.goal_sent = True

        elif self.robot_status=="Quest":
            # 검색 순회 로직. ex) 5구역 순서대로
            pass

        elif self.robot_status=="Tracking":
            # bbox_callback에서 제어 중
            pass
        elif self.robot_status=="READY":
            # 대기
            pass
        else:
            # 기타 상태
            pass

    def publish_cmd_vel(self, lx, az):
        twist = Twist()
        twist.linear.x = lx
        twist.angular.z = az
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info(f"[MOVE] cmd_vel: linear={lx:.3f}, angular={az:.3f}")

########################################
# main: Spin both nodes with MultiThreadedExecutor
########################################

def main(args=None):
    rclpy.init(args=args)

    yolo_node = YoloCenterPublisher()
    move_node = MoveNode()

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(yolo_node)
    executor.add_node(move_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        yolo_node.get_logger().info("Shutting down integrated system (yolo_node + move_node).")
    finally:
        executor.shutdown()
        yolo_node.destroy_node()
        move_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
