import os
import yaml
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Bool, Float32
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel, QProgressBar, QTextEdit
from PyQt5.QtGui import QPixmap, QImage
from PyQt5.QtCore import QThread, pyqtSignal, QTimer
import sys
import cv2
import numpy as np
from cv_bridge import CvBridge
import time
import threading

class ROS2Thread(QThread):
    battery_signal = pyqtSignal(float)
    camera_signal_tb1 = pyqtSignal(np.ndarray)
    camera_signal_tb2 = pyqtSignal(np.ndarray)
    map_signal = pyqtSignal(np.ndarray)
    # 정찰 로봇(tb1)의 좌표(x, y)를 전달하는 시그널 (배터리 임계치 도달 시 구조 로봇 명령 연동)
    robot_position_signal = pyqtSignal(float, float)

    def __init__(self):
        super().__init__()
        rclpy.init()
        self.node = Node('gui')
        self.bridge = CvBridge()  
        
        # 배터리 관리 변수
        self.battery_level = 100  # 초기 배터리 상태 100%
        self.battery_threshold = 10  # 10% 임계치
        self.battery_timer = QTimer()
        self.battery_timer.timeout.connect(self.reduce_battery)
        self.battery_timer.start(500)  # 1초마다 배터리 감소
        
###############################경로변경################################
        # 맵 파일 경로 (환경에 맞게 수정)
        self.yaml_file = '/home/ryosang/Downloads/map.yaml'
        self.pgm_file = '/home/ryosang/Downloads/map.pgm'

        self.map_img = None
        self.resolution = 0.05
        self.origin_x = -2.69
        self.origin_y = -3.58
        self.width = 0
        self.height = 0

        self.robot_x = None
        self.robot_y = None

        self.load_map()

        # ROS2 토픽 구독 및 퍼블리셔 설정
        self.odom_subscriber = self.node.create_subscription(Odometry, '/tb1/odom', self.odom_callback, 10)
        self.battery_subscription = self.node.create_subscription(Float32, '/scout/battery', self.update_battery, 10)
        self.camera_subscription_tb1 = self.node.create_subscription(Image, '/tb1/camera/image_raw', self.update_camera_tb1, 10)
        self.camera_subscription_tb2 = self.node.create_subscription(Image, '/tb2/camera/image_raw', self.update_camera_tb2, 10)

        self.publisher_run = self.node.create_publisher(Bool, '/gui/run', 10)
        self.publisher_stop = self.node.create_publisher(Bool, '/gui/stop', 10)
        self.publisher_recover = self.node.create_publisher(Bool, '/gui/recover', 10)
        self.publisher_cmd_vel_tb1 = self.node.create_publisher(Twist, '/tb1/cmd_vel', 10)
        self.publisher_cmd_vel_tb2 = self.node.create_publisher(Twist, '/tb2/cmd_vel', 10)
        self.goal_pub = self.node.create_publisher(PoseStamped, '/tb2/goal_pose', 10)
        self.last_time = time.time()
        self.frame_skip_interval = 1 / 10  # 10FPS 제한

        self.ros_spin_thread = threading.Thread(target=self.ros_spin, daemon=True)
        self.ros_spin_thread.start()  # ROS2 이벤트 루프 별도 스레드 실행

        # 구조 명령 중복 방지를 위한 플래그
        self.rescue_sent = False

    def ros_spin(self):
        """ROS2 이벤트 루프 실행"""
        while rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.1)

    def load_map(self):
        """PGM 맵을 불러와서 OpenCV로 처리"""
        if not os.path.exists(self.yaml_file) or not os.path.exists(self.pgm_file):
            self.node.get_logger().error("맵 파일을 찾을 수 없습니다!")
            return

        with open(self.yaml_file, 'r') as yf:
            ycontent = yaml.safe_load(yf)
            self.resolution = float(ycontent['resolution'])
            origin = ycontent['origin']
            self.origin_x = float(origin[0])
            self.origin_y = float(origin[1])

        img_gray = cv2.imread(self.pgm_file, cv2.IMREAD_UNCHANGED)
        if img_gray is None:
            self.node.get_logger().error("맵 이미지를 로드하는 데 실패했습니다!")
            return

        self.height, self.width = img_gray.shape[:2]
        self.map_data = img_gray.astype(np.uint8)

        # 맵 색상 변환
        self.map_data = np.where(self.map_data == 205, 127, self.map_data)  # 미탐색 → 회색
        self.map_data = np.where(self.map_data == 254, 255, self.map_data)  # 빈 공간 → 흰색
        self.map_data = np.where(self.map_data == 0, 0, self.map_data)      # 장애물 → 검정색

        # 컬러 맵 생성
        self.map_img = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        for i in range(self.height):
            for j in range(self.width):
                val = self.map_data[i, j]
                if val == 0:
                    self.map_img[i, j] = (0, 0, 0)
                elif val == 127:
                    self.map_img[i, j] = (127, 127, 127)
                elif val == 255:
                    self.map_img[i, j] = (255, 255, 255)

        self.node.get_logger().info(f"맵 로드 완료: 크기=({self.width}x{self.height}), 해상도={self.resolution}, "
                                    f"원점=({self.origin_x:.2f}, {self.origin_y:.2f})")

    def odom_callback(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x + 2
        self.robot_y = msg.pose.pose.position.y + 0.5

    def update_battery(self, msg):
        self.battery_signal.emit(msg.data)

    def update_camera_tb1(self, msg):
        """카메라 데이터를 수신하여 OpenCV 이미지로 변환"""
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            if img is not None:
                self.camera_signal_tb1.emit(img)
        except Exception as e:
            self.node.get_logger().error(f"카메라1 업데이트 오류: {e}")

    def update_camera_tb2(self, msg):
        """카메라 데이터를 수신하여 OpenCV 이미지로 변환"""
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            if img is not None:
                self.camera_signal_tb2.emit(img)
        except Exception as e:
            self.node.get_logger().error(f"카메라2 업데이트 오류: {e}")

    def send_cmd_vel(self, robot='tb1', linear_x=0.0, angular_z=0.0):
        """특정 로봇에 이동 명령을 보내는 함수 (일정 횟수 반복)"""
        def publish_cmd():
            twist = Twist()
            twist.linear.x = float(linear_x)
            twist.angular.z = float(angular_z)

            if robot == 'tb1':
                for _ in range(10):
                    self.publisher_cmd_vel_tb1.publish(twist)
                    time.sleep(0.1)
            elif robot == 'tb2':
                for _ in range(10):
                    self.publisher_cmd_vel_tb2.publish(twist)
                    time.sleep(0.1)
            else:
                self.node.get_logger().warn(f"잘못된 로봇 이름: {robot}")

        threading.Thread(target=publish_cmd, daemon=True).start()

    def send_cmd_vel_nav2(self):
        if self.robot_x is None:
            self.node.get_logger().warn('Current pose not available yet.')
            return
        
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.node.get_clock().now().to_msg()
        goal_msg.header.frame_id = 'map'
        # goal_msg.pose = self.current_pose
        goal_msg.pose.position.x = self.robot_x
        goal_msg.pose.position.y = self.robot_y
        
        self.goal_pub.publish(goal_msg)
        self.node.get_logger().info(
            f'Rescue goal sent to rescue robot: (x: {self.robot_x:.2f}, y: {self.robot_y:.2f})'
        )

    def update_visualization(self):
        """OpenCV를 이용한 실시간 맵 업데이트"""
        if self.map_img is None:
            self.node.get_logger().error("맵이 로드되지 않았습니다!")
            return

        vis_img = self.map_img.copy()

        if self.robot_x is not None and self.robot_y is not None:
            # 월드 좌표를 맵 픽셀 좌표로 변환
            pixel_x = int((self.robot_x - self.origin_x) / self.resolution)
            pixel_y = int((self.robot_y - self.origin_y) / self.resolution)
            pixel_y = self.height - pixel_y  # OpenCV 기준 y축 반전

            if 0 <= pixel_x < self.width and 0 <= pixel_y < self.height:
                cv2.circle(vis_img, (pixel_x, pixel_y), 5, (0, 255, 0), -1)
            else:
                self.node.get_logger().warn(f"로봇 위치가 맵 바깥에 있음: ({pixel_x}, {pixel_y})")

        self.map_signal.emit(vis_img)

    def reduce_battery(self):
        """1초마다 배터리 잔량 1%씩 감소, 임계치 도달 시 정찰 로봇 정지 및 구조 로봇 명령 전송"""
        if self.battery_level > 0:
            self.battery_level -= 1
            self.battery_signal.emit(self.battery_level)

        if self.battery_level == self.battery_threshold and not self.rescue_sent:
            self.rescue_sent = True
            # 정찰 로봇 정지 명령
            self.send_cmd_vel(robot='tb1', linear_x=0.0, angular_z=0.0)
            self.node.get_logger().warn(f"배터리 부족! tb1 좌표: x={self.robot_x:.2f}, y={self.robot_y:.2f}")
            # 로봇 좌표가 있을 경우 구조 로봇으로 좌표 전송
            if self.robot_x is not None and self.robot_y is not None:
                self.robot_position_signal.emit(self.robot_x, self.robot_y)
                # 구조 로봇 tb2가 목표 위치로 이동하도록 명령
                self.send_cmd_vel_nav2()
            else:
                self.node.get_logger().warn("현재 로봇 좌표가 확보되지 않음!")


class GUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Rokey 달 탐사 화면")
        self.setGeometry(100, 100, 1200, 700)
        self.layout = QVBoxLayout()

        # 카메라 화면
        self.top_layout = QHBoxLayout()
        self.camera_label_tb1 = QLabel("정찰 로봇1 카메라 화면")
        self.camera_label_tb1.setFixedSize(400, 400)
        self.camera_label_tb2 = QLabel("정찰 로봇2 카메라 화면")
        self.camera_label_tb2.setFixedSize(400, 400)
        self.top_layout.addWidget(self.camera_label_tb1)
        self.top_layout.addWidget(self.camera_label_tb2)
        self.layout.addLayout(self.top_layout)

        # 배터리 상태
        self.battery_bar = QProgressBar()
        self.battery_bar.setRange(0, 100)
        self.battery_bar.setValue(100)
        self.layout.addWidget(QLabel("정찰 로봇 배터리 상황"))
        self.layout.addWidget(self.battery_bar)

        # 로봇 제어 버튼
        self.control_layout = QHBoxLayout()
        self.forward_button = QPushButton("▲ 전진")
        self.backward_button = QPushButton("▼ 후진")
        self.left_button = QPushButton("◀ 좌회전")
        self.right_button = QPushButton("▶ 우회전")
        self.stop_motion_button = QPushButton("■ 정지")

        self.control_layout.addWidget(self.left_button)
        self.control_layout.addWidget(self.forward_button)
        self.control_layout.addWidget(self.stop_motion_button)
        self.control_layout.addWidget(self.backward_button)
        self.control_layout.addWidget(self.right_button)
        self.layout.addLayout(self.control_layout)

        # 로그창
        self.log_label = QLabel("로그")
        self.log_box = QTextEdit()
        self.log_box.setReadOnly(True)
        self.layout.addWidget(self.log_label)
        self.layout.addWidget(self.log_box)

        # 맵 뷰 (map_signal을 받아 QLabel에 표시)
        self.mapping_label = QLabel("맵 뷰")
        self.mapping_label.setFixedSize(200, 200)
        self.layout.addWidget(self.mapping_label)

        self.setLayout(self.layout)

        # ROS2 쓰레드 실행 및 시그널 연결
        self.ros2_thread = ROS2Thread()
        self.ros2_thread.battery_signal.connect(self.update_battery_bar)
        self.ros2_thread.camera_signal_tb1.connect(self.update_camera_view_tb1)
        self.ros2_thread.camera_signal_tb2.connect(self.update_camera_view_tb2)
        self.ros2_thread.map_signal.connect(self.update_map_view)
        self.ros2_thread.robot_position_signal.connect(self.send_robot_to_rescue)
        self.ros2_thread.start()

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.ros2_thread.update_visualization)
        self.timer.start(100)
        
        # 버튼 이벤트 연결
        self.forward_button.pressed.connect(lambda: self.move_robot(0.2, 0))
        self.backward_button.pressed.connect(lambda: self.move_robot(-0.2, 0))
        self.left_button.pressed.connect(lambda: self.move_robot(0, 0.5))
        self.right_button.pressed.connect(lambda: self.move_robot(0, -0.5))
        self.stop_motion_button.clicked.connect(lambda: self.move_robot(0, 0))

    def move_robot(self, linear_x, angular_z):
        """GUI 버튼을 누르면 ROS2로 이동 명령을 Publish"""
        self.ros2_thread.send_cmd_vel(robot='tb1', linear_x=linear_x, angular_z=angular_z)
        self.log_box.append(f"로봇 이동 명령: 선속도 {linear_x}, 각속도 {angular_z}")

    def update_battery_bar(self, value):
        self.battery_bar.setValue(int(value))
        self.log_box.append(f"배터리 상태: {value}%")

    def update_camera_view_tb1(self, img):
        height, width, channel = img.shape
        bytes_per_line = 3 * width
        qimg = QImage(img.data, width, height, bytes_per_line, QImage.Format_RGB888)
        self.camera_label_tb1.setPixmap(QPixmap.fromImage(qimg))

    def update_camera_view_tb2(self, img):
        height, width, channel = img.shape
        bytes_per_line = 3 * width
        qimg = QImage(img.data, width, height, bytes_per_line, QImage.Format_RGB888)
        self.camera_label_tb2.setPixmap(QPixmap.fromImage(qimg))

    def update_map_view(self, img):
        """맵 이미지를 QLabel에 표시"""
        if img is None or img.size == 0:
            self.log_box.append("맵 이미지 오류!")
            return

        height, width, _ = img.shape
        bytes_per_line = 3 * width
        qimg = QImage(img.data, width, height, bytes_per_line, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(qimg)
        self.mapping_label.setPixmap(pixmap)
        self.mapping_label.update()

    def send_robot_to_rescue(self, x, y):
        """배터리가 10%가 되면 tb2가 tb1의 위치로 이동 명령 실행"""
        self.log_box.append(f"배터리 부족! tb1 좌표 ({x:.2f}, {y:.2f})에서 정지. tb2 이동 시작.")
        # tb2가 tb1의 위치로 이동하도록 간단히 전진 명령을 보내는 예시 (필요시 경로 계획 알고리즘 적용)
        # self.ros2_thread.send_cmd_vel(robot='tb2', linear_x=0.2, angular_z=0.0)
        self.ros2_thread.send_cmd_vel_nav2()


def main():
    app = QApplication(sys.argv)
    gui = GUI()
    gui.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
