#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from aruco_msgs.msg import MarkerArray
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Pose, PoseArray

# arm control
from turtlebot_cosmo_interface.srv import MoveitControl
from srv_call_test import TurtlebotArmClient

import time
import json  # 변경: ast 대신 json 사용

class IntegratedProcess(Node):
    def __init__(self):
        super().__init__('integrated_process')

        # ------------------------------
        # Subscribers
        # ------------------------------
        # (1) Aruco Marker
        self.aruco_sub = self.create_subscription(
            MarkerArray,
            'detected_markers',
            self.aruco_listener_callback,
            10
        )
        
        # (2) YOLO Detection
        self.yolo_sub = self.create_subscription(
            String,
            '/yolo/detected_info',
            self.yolo_listener_callback,
            10
        )
        
        # ------------------------------
        # Publisher
        # ------------------------------
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 2)
        
        # ------------------------------
        # State Variables
        # ------------------------------
        self.aruco_marker_found = False
        self.task_completed = False
        self.yolofind = False
        self.armrun = False

        # YOLO x,y 좌표 (사용자 코드: data_list[0][1], data_list[0][2] 형식)
        self.yolo_x = 0.0
        self.yolo_y = 0.0
        
        self.marker_id = None
        self.state = 'START'  
        self.count = 0

        # 아루코 마커 Pose (odom 기준)
        self.aruco_pose = None

        # ------------------------------
        # Main Timer (1Hz)
        # ------------------------------
        self.create_timer(1.0, self.run_tasks)

    # ------------------------------
    # Aruco Listener Callback
    # ------------------------------
    def aruco_listener_callback(self, msg: MarkerArray):
        # 상태가 ARUCO / BACKWARD / CHECK 일 때만 처리
        if self.state not in ('ARUCO', 'BACKWARD', 'CHECK'):
            return

        target_marker_id = 0  # 0번 마커를 찾는다
        
        for marker in msg.markers:
            if marker.id == target_marker_id:
                # 0번 마커를 찾았다면
                self.marker_id = marker.id
                self.aruco_pose = marker.pose.pose
                self.get_logger().info(
                    f'Marker ID: {marker.id}, PositionZ: {self.aruco_pose.position.z}'
                )
                self.aruco_marker_found = True

                # 현재 상태에 따라 전진 / 후진 함수를 호출
                if self.state == 'ARUCO':
                    self.execute_forward_task(self.aruco_pose.position.z)
                elif self.state == 'BACKWARD':
                    self.execute_backward_task(self.aruco_pose.position.z)
            
            # CHECK 상태일 때, marker3 근접 확인
            if self.state == 'CHECK':
                if marker.id == 3:
                    x_position = marker.pose.pose.position.x
                    # marker3이 x=±0.05 이내면 정지
                    if abs(x_position) <= 0.05:
                        print("find marker3 => stop")
                        self.publish_cmd_vel(0.0)
                        self.final_task()
                    else:
                        print("keep run => forward..")
                        self.publish_cmd_vel(0.03)

    # ------------------------------
    # YOLO Listener Callback
    # ------------------------------
    def yolo_listener_callback(self, msg: String):
        # 상태가 YOLO / PURPLE 일 때만 처리
        if self.state not in ('YOLO', 'PURPLE'):
            return

        # 로봇 암이 동작 중이면 무시
        if self.armrun:
            return

        # YOLO 데이터 파싱
        # 가정: msg.data = "[['colorName', x_val, y_val], ...]"
        try:
            data_str = msg.data
            data_list = json.loads(data_str)  # 예: [['red', 0.12, -0.04], ['blue', 0.07, 0.03]]

            if len(data_list) > 0:
                # 첫 번째 객체
                self.yolo_x = float(data_list[0][1])  # x 좌표
                self.yolo_y = float(data_list[0][2])  # y 좌표

                print(f"Detected coordinates: {self.yolo_x}, {self.yolo_y}")
                print("done")

                # 상태에 따라 로봇 암 제어
                if self.state == 'YOLO':
                    if not self.yolofind:
                        self.yolofind = True
                        self.yolo_arm_controll()
                        
                        if self.count == 1:
                            self.home2_arm_controll()
                            self.state = 'BACKWARD'
                
                elif self.state == 'PURPLE':
                    if not self.yolofind:
                        self.yolofind = True
                        self.purple_arm_control()

        except Exception as e:
            self.get_logger().error(f"Error processing the data: {e}")

    # ------------------------------
    # State Machine (Timer)
    # ------------------------------
    def run_tasks(self):
        if self.state == 'START':
            self.execute_aruco_task()
        elif self.state == 'FINISH':
            self.finish_task()

    # ------------------------------
    # 1) START -> ARUCO
    # ------------------------------
    def execute_aruco_task(self):
        self.state = 'ARUCO'
        
    # ------------------------------
    # 전진 함수
    # ------------------------------
    def execute_forward_task(self, current_z_position):
        # 0.3m 까지 전진
        if self.aruco_marker_found and self.aruco_pose:
            self.get_logger().info("Executing forward task...")
            if current_z_position > 0.3:
                self.publish_cmd_vel(0.05)
            elif current_z_position > 0.25:
                self.publish_cmd_vel(0.025)
            else:
                self.publish_cmd_vel(0.0)
                self.get_logger().info("Target reached => camera_arm_controll => YOLO")
                self.camera_arm_controll()
                self.state = 'YOLO'

    # ------------------------------
    # 후진 함수
    # ------------------------------
    def execute_backward_task(self, current_z_position):
        # 1m 후진
        if self.aruco_marker_found and self.aruco_pose:
            self.get_logger().info("Executing backward task...")
            if current_z_position < 0.98:
                self.publish_cmd_vel(-0.05)
            else:
                self.publish_cmd_vel(0.0)
                self.get_logger().info("Target reached => box_home_arm_controll => PURPLE")
                self.box_home_arm_controll()
                self.state = 'PURPLE'

    # ------------------------------
    # 카메라 포즈
    # ------------------------------
    def camera_arm_controll(self):
        arm_client = TurtlebotArmClient()
        response = arm_client.send_request(1, "camera_home")
        arm_client.get_logger().info(f'Response: {response.response}')
        time.sleep(3)

    # ------------------------------
    # home2 포즈
    # ------------------------------
    def home2_arm_controll(self):
        arm_client = TurtlebotArmClient()
        response = arm_client.send_request(1, "home2")
        arm_client.get_logger().info(f'Response: {response.response}')
        time.sleep(3)

    # ------------------------------
    # box_home_01 포즈
    # ------------------------------
    def box_home_arm_controll(self):
        arm_client = TurtlebotArmClient()
        response = arm_client.send_request(1, "box_home_01")
        arm_client.get_logger().info(f'Response: {response.response}')
        time.sleep(3)

    # ------------------------------
    # PoseArray 편의 함수
    # ------------------------------
    def append_pose_init(self, x, y, z):
        pose_array = PoseArray()
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose_array.poses.append(pose)
        return pose_array

    # ------------------------------
    # YOLO 로봇 팔 제어
    # ------------------------------
    def yolo_arm_controll(self):
        arm_client = TurtlebotArmClient()

        print("task start!")
        print(f"Get coordinates: {self.yolo_x}, {self.yolo_y}")

        if self.yolofind:
            self.armrun = True

            resp = arm_client.send_request(2, "open") 
            arm_client.get_logger().info(f'Response: {resp.response}')
            time.sleep(1)

            # 예시 (기존 코드를 유지)
            pose_array = self.append_pose_init(
                0.137496 - self.yolo_y + 0.05,
                0.00 - self.yolo_x,
                0.122354
            )
            resp = arm_client.send_request(0, "", pose_array)
            arm_client.get_logger().info(f'Response: {resp.response}')

            pose_array = self.append_pose_init(
                0.137496 - self.yolo_y + 0.05,
                0.00 - self.yolo_x,
                0.087354
            )
            resp = arm_client.send_request(0, "", pose_array)
            arm_client.get_logger().info(f'Response: {resp.response}')

            resp = arm_client.send_request(2, "close")
            arm_client.get_logger().info(f'Response: {resp.response}')

            resp = arm_client.send_request(1, "home2")
            arm_client.get_logger().info(f'Response: {resp.response}')
            time.sleep(1)

            print("conveyor task start")
            resp = arm_client.send_request(1, "conveyor_up")
            resp = arm_client.send_request(1, "test_conveyor")
            resp = arm_client.send_request(2, "open")

            print("throw")
            resp = arm_client.send_request(1, "conveyor_up")
            resp = arm_client.send_request(1, "camera_home")
            time.sleep(3)

            print("jobs_done")

            self.armrun = False
            self.yolofind = False
            self.count += 1

    # ------------------------------
    # 보라색 로봇 제어
    # ------------------------------
    def purple_arm_control(self):
        if self.state == 'PURPLE':
            arm_client = TurtlebotArmClient()

            print("task start!")
            print(f"Get coordinates: {self.yolo_x}, {self.yolo_y}")

            if self.yolofind:
                self.armrun = True

                resp = arm_client.send_request(2, "open")
                arm_client.get_logger().info(f'Response: {resp.response}')
                time.sleep(1)

                pose_array = self.append_pose_init(
                    0.0101294 - self.yolo_x,
                    -0.2800000,
                    0.205779 - self.yolo_y + 0.06
                )
                resp = arm_client.send_request(3, "", pose_array)
                arm_client.get_logger().info(f'Response: {resp.response}')

                resp = arm_client.send_request(9, "")
                arm_client.get_logger().info(f'Response: {resp.response}')

                pose_array = self.append_pose_init(
                    0.0101294 - self.yolo_x,
                    -0.3100000,
                    0.205779 - self.yolo_y + 0.06
                )
                resp = arm_client.send_request(3, "", pose_array)
                arm_client.get_logger().info(f'Response: {resp.response}')

                resp = arm_client.send_request(9, "")
                arm_client.get_logger().info(f'Response: {resp.response}')

                resp = arm_client.send_request(2, "close")
                arm_client.get_logger().info(f'Response: {resp.response}')
                time.sleep(1)

                resp = arm_client.send_request(1, "box_up_01")
                time.sleep(1)
                resp = arm_client.send_request(1, "box_up_02")
                time.sleep(1)
                resp = arm_client.send_request(1, "box_up_03")
                time.sleep(1)
                resp = arm_client.send_request(1, "box_back_01")
                time.sleep(1)

                print("jobs_done")

                self.armrun = False
                self.yolofind = False
                self.state = 'CHECK'

    # ------------------------------
    # 마지막 작업
    # ------------------------------
    def final_task(self):
        arm_client = TurtlebotArmClient()
        resp = arm_client.send_request(1, "box_back_put")
        time.sleep(1)
        resp = arm_client.send_request(2, "open")
        self.state = "FINISH"

    # ------------------------------
    # 모든 작업 완료
    # ------------------------------
    def finish_task(self):
        if self.state == 'FINISH':
            self.get_logger().info("All tasks are complete!")
            self.destroy_node()
            rclpy.shutdown()

    # ------------------------------
    # cmd_vel 발행 함수
    # ------------------------------
    def publish_cmd_vel(self, linear_x, angular_z=0.0):
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.cmd_vel_publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = IntegratedProcess()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
