#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, Quaternion
import json
import time

class MoveNode(Node):
    def __init__(self):
        super().__init__('move_node')
        # GUI로부터 로봇 상태(robot_status) 수신 ("Detected", "Quest", "Tracking", "READY")
        self.create_subscription(String, 'robot_status', self.robot_status_callback, 10)
        # CCTV에서 전달되는 위험 알람 메시지 수신 (예: {"detected": true})
        self.create_subscription(String, 'result_Yolo_CCTV', self.result_yolo_cctv_callback, 10)
        # Tracking 모드에서 YOLO가 전달하는 bbox 정보 수신 (예: {"bbox": {"x1":..., "y1":..., "x2":..., "y2":...}})
        self.create_subscription(String, 'result_Yolo_AMR', self.result_yolo_amr_callback, 10)

        # 초기 포즈 퍼블리셔 (/initialpose)
        self.pose_publisher = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        # 로봇 속도 명령 퍼블리셔 (cmd_vel)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        # 상태 기반 타이머 (1초 주기)
        self.create_timer(1.0, self.timer_callback)

        # 내부 상태 변수
        self.robot_status = None    # "Detected", "Quest", "Tracking", "READY" 중 하나
        self.goal_sent = False      # "Detected" 상태에서 목표를 한 번만 전송하기 위한 플래그
        self.quest_goal_index = 0   # "Quest" 모드에서 순차적으로 목표 전송을 위한 인덱스

        # 미리 정의된 6구역 좌표 (x, y, theta)
        self.nav_points = [
            {"x": 0.20662596156342655, "y": -0.4871472785139167, "theta": -1.17},
            {"x": -0.5098931522807715, "y": -0.5766481689018857, "theta": 3.017},
            {"x": -0.7339190729114683, "y": -0.366880572197456, "theta": 2.384},
            {"x": -1.2838784688896316, "y": -0.2265589907963823, "theta": 2.893},
            {"x": -1.4204211936158202, "y": -0.5974287556500998, "theta": -1.930},
            {"x": -1.5243525436303194, "y": -0.6399604662931665, "theta": -2.748}
        ]
        # 추적 제어를 위한 화면 중앙 (예: 640x480 해상도)
        self.image_center = (320, 240)
    
    def robot_status_callback(self, msg: String):
        """GUI로부터 로봇 상태 업데이트"""
        self.robot_status = msg.data
        self.get_logger().info(f"Robot_status updated: {self.robot_status}")
        if self.robot_status in ["Detected", "Quest"]:
            self.goal_sent = False
            self.quest_goal_index = 0

    def result_yolo_cctv_callback(self, msg: String):
        """
        CCTV 알람 메시지 처리 (예: {"detected": true})
        위험 상황 발생 시 로봇이 'Detected' 상태로 전환하도록 할 수 있음.
        """
        try:
            data = json.loads(msg.data)
        except Exception as e:
            self.get_logger().error("Failed to parse result_yolo_CCTV JSON")
            return
        if data.get("detected", False):
            self.get_logger().info("CCTV: 객체 검출됨")
            # 필요에 따라 상태 전환 처리 가능 (예: self.robot_status = "Detected")

    def result_yolo_amr_callback(self, msg: String):
        """
        Tracking 모드에서 YOLO가 전달한 bbox 정보를 처리하여,
        화면 중앙과의 오차를 기반으로 로봇의 이동(cmd_vel) 제어를 수행.
        JSON 예시: {"bbox": {"x1": value, "y1": value, "x2": value, "y2": value}}
        """
        if self.robot_status != "Tracking":
            return  # Tracking 상태가 아니면 무시
        
        try:
            data = json.loads(msg.data)
        except Exception as e:
            self.get_logger().error("Failed to parse result_yolo_AMR JSON")
            return
        
        bbox = data.get("bbox", None)
        if bbox is None:
            self.get_logger().warn("result_yolo_AMR 메시지에 bbox 정보가 없습니다.")
            return
        
        # 바운딩 박스 좌표 추출 및 중심 계산
        x1 = bbox.get("x1", 0)
        y1 = bbox.get("y1", 0)
        x2 = bbox.get("x2", 0)
        y2 = bbox.get("y2", 0)
        cx = (x1 + x2) / 2.0
        cy = (y1 + y2) / 2.0
        self.get_logger().info(f"AMR bbox center: ({cx:.1f}, {cy:.1f})")

        # 화면 중앙과 비교하여 제어 (간단한 사분면 기준)
        image_cx, image_cy = self.image_center
        threshold = 40  # 픽셀 단위 오차 허용치
        linear_x = 0.0
        angular_z = 0.0

        # 좌우 오차에 따른 회전 제어
        if cx < image_cx - threshold:
            angular_z = 0.5  # 왼쪽 회전
        elif cx > image_cx + threshold:
            angular_z = -0.5  # 오른쪽 회전
        else:
            angular_z = 0.0

        # bbox 높이로부터 거리 판단 (높이가 작으면 멀리, 크면 가까움)
        height = (y2 - y1)
        if height < 140:
            linear_x = 0.10  # 전진 속도 (멀리 있을 때)
        elif height > 180:
            linear_x = -0.05  # 후진 또는 감속 (너무 가까울 때)
        else:
            linear_x = 0.0  # 조금 전진

        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info(f"cmd_vel 퍼블리시: linear_x={linear_x}, angular_z={angular_z}")

    def timer_callback(self):
        """
        1초 주기로 현재 로봇 상태에 따라 동작 수행:
          - "Detected": 미리 정의된 구역(예: 첫 번째 좌표)으로 목표 전송
          - "Quest": 구역을 순차적으로 탐색
          - "Tracking": bbox 기반 제어는 result_yolo_AMR_callback에서 처리
          - "READY": 초기 포즈 발행
        """
        if self.robot_status == "Detected":
            if not self.goal_sent:
                target = self.nav_points[0]  # 예제: 첫 번째 좌표 선택
                self.send_nav_goal(target)
                self.goal_sent = True
        elif self.robot_status == "Quest":
            if self.quest_goal_index < len(self.nav_points):
                target = self.nav_points[self.quest_goal_index]
                self.send_nav_goal(target)
                self.quest_goal_index += 1
            else:
                self.get_logger().info("Quest: 모든 구역 탐색 완료")
        elif self.robot_status == "Tracking":
            # Tracking 상태는 result_yolo_amr_callback에서 처리
            pass
        elif self.robot_status == "READY":
            self.publish_initial_pose()
        else:
            self.get_logger().info("유효한 Robot_status가 설정되지 않음")

    def send_nav_goal(self, target):
        """
        nav2에 목표 좌표를 전송하는 함수 (여기서는 로그 출력으로 대체)
        target 예시: {"x": value, "y": value, "theta": value}
        """
        self.get_logger().info(f"nav2 Goal 전송: x={target['x']}, y={target['y']}, theta={target['theta']}")
        # 실제 nav2 Action Client나 Service Call을 사용하여 구현 가능

    def publish_initial_pose(self):
        """
        초기 포즈 메시지를 /initialpose 토픽으로 퍼블리시하여 로봇의 초기 위치를 설정
        """
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.get_clock().now().to_msg()
        initial_pose.pose.pose.position.x = 0.06553493106365204
        initial_pose.pose.pose.position.y = 0.06547150760889053
        initial_pose.pose.pose.position.z = 0.0
        initial_pose.pose.pose.orientation = Quaternion(
            x=0.0,
            y=0.0,
            z=-0.009319258229818488,
            w=0.9999565747701477
        )
        initial_pose.pose.covariance = [
            0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891909122467
        ]
        self.pose_publisher.publish(initial_pose)
        self.get_logger().info('Initial pose published.')

def main(args=None):
    rclpy.init(args=args)
    node = MoveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("move_node 종료")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
