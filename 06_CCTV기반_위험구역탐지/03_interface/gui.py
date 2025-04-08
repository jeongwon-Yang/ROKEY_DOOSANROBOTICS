import sys
import rclpy
from rclpy.node import Node
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QLabel, QPushButton, QVBoxLayout, QHBoxLayout, QGridLayout
from PyQt5.QtGui import QPixmap, QImage
from PyQt5.QtCore import pyqtSignal, QObject, Qt, QThread
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String, Float32MultiArray
from cv_bridge import CvBridge
import cv2
from threading import Thread
import json
class ROS2Bridge(QObject):
    cam1_signal = pyqtSignal(object)
    cam2_signal = pyqtSignal(object)
    status_signal = pyqtSignal(str)
class Server(Node):
    def __init__(self, bridge_obj):
        super().__init__('ros2_gui_node')
        self.bridge = CvBridge()
        self.qt_bridge = bridge_obj
        self.pub_status = self.create_publisher(String, 'robot_status', 10)
        self.sub_status = self.create_subscription(String, 'robot_status', self.set_robot_status, 10)
        # 현재 로봇 상태 초기화
        self.current_status = "READY"
        # 상태를 주기적으로 업데이트하는 타이머 (GUI와 연동)
        self.timer = self.create_timer(1.0, self.update_status)  # 1초마다 상태 업데이트
        self.sub_cctv_yolo = self.create_subscription(CompressedImage, 'Img_Yolo_cctv', self.cctv_Yolo_callback, 10)
        self.sub_cctv_result = self.create_subscription(String, 'result_Yolo_cctv', self.cctv_result_callback, 10)
        self.sub_AMR_yolo = self.create_subscription(CompressedImage, 'Img_Yolo_AMR', self.AMR_Yolo_callback, 10)
        self.sub_AMR_result = self.create_subscription(Float32MultiArray, 'bounding_box/center', self.AMR_result_callback, 10)
        self.is_running = False
    def cctv_Yolo_callback(self, msg):
        if self.is_running:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.qt_bridge.cam1_signal.emit(cv_image)
    def AMR_Yolo_callback(self, msg):
        if self.is_running:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.qt_bridge.cam2_signal.emit(cv_image)
    def cctv_result_callback(self, msg):
        # CCTV 데이터 수신할 경우 DETECTED 상태 변경
        try:
            # JSON 문자열을 파이썬 딕셔너리로 변환
            data = json.loads(msg.data)
            # 'goal' 키가 존재하면 x, y 값을 추출
            if 'goal' in data:
                x = float(data['goal']['x'])
                y = float(data['goal']['y'])
                self.get_logger().info(f"Received Goal Coordinates: x={x}, y={y}")
            if 'rgb_values' in data:
                r_value = int(data['rgb_values']["R"])
                g_value = int(data['rgb_values']["G"])
                b_value = int(data['rgb_values']["B"])
                self.get_logger().info(f"Received Color Value: R={r_value}, G={g_value}, B={b_value}")
            # 상태를 "QUEST"로 변경
            self.set_robot_status("DETECTED")
        except json.JSONDecodeError:
            self.get_logger().error("Failed to decode JSON data.")
    def AMR_result_callback(self, msg):
        # AMR 바운딩박스 센터값 수신할 경우 TRACKING으로 상태 변경
        try:
            if msg:
                self.set_robot_status("TRACKING")
        except:
            self.get_logger().error("Failed to receive result_Yolo_AMR")
    def process_cctv_result(self, x, y):
        """ x, y 좌표 확인 및  """
        self.get_logger().info(f"Processing Goal: x={x}, y={y}")
    def send_command(self, command):
        msg = String()
        msg.data = command
        self.publisher.publish(msg)
        print(f"Sent command: {command}")
    def start_subscription(self):
        self.is_running = True
    def stop_subscription(self):
        self.is_running = False
        # 상태를 "QUEST"로 변경
        self.set_robot_status("READY")
################# 로봇 상태 변경 부분 ###############
    def set_robot_status(self, status):
        # self.get_logger().info(f"set_robot_status check")
        if status not in ["READY", "DETECTED", "QUEST", "TRACKING"]:
            return
        # 상태가 변경된 경우에만 상태를 publish
        if status != self.current_status:
            self.current_status = status
            msg = String()
            msg.data = status
            self.pub_status.publish(msg)
            # 상태 변경 시 GUI에도 전달
            self.qt_bridge.status_signal.emit(status)
            self.get_logger().info(f"{status}로 상태 변경")
    def update_status(self):
        """ 현재 로봇 상태를 ROS2에 지속적으로 publish하고, 상태가 바뀌었을 때만 GUI를 업데이트 """
        msg = String()
        msg.data = self.current_status
        self.pub_status.publish(msg)  # 1초마다 ROS2에 상태 publish
        # GUI 상태가 변할 때만 업데이트 (중복 업데이트 방지)
        if msg.data != getattr(self, "last_gui_status", None):
            self.qt_bridge.status_signal.emit(msg.data)  # GUI에 변경된 상태만 업데이트
            self.last_gui_status = msg.data  # 마지막으로 GUI에 표시된 상태 저장
###################################################
class ROSThread(QThread):
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
    def run(self):
        rclpy.spin(self.ros_node)
    def stop(self):
        self.quit()
        self.wait()
class MainWindow(QMainWindow):
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.init_ui()
    def init_ui(self):
        self.setWindowTitle('H.E.R.O')
        self.setMinimumSize(1440, 720)
        # 중앙 위젯과 메인 레이아웃
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)
        # 상단 제목 레이아웃
        title_layout = QHBoxLayout()
        self.cctv_label = QLabel('CCTV 화면')
        self.cctv_label.setAlignment(Qt.AlignCenter)
        self.cctv_label.setStyleSheet("font-size: 48px; font-weight: bold;")
        self.robot_label = QLabel('로봇 시점')
        self.robot_label.setAlignment(Qt.AlignCenter)
        self.robot_label.setStyleSheet("font-size: 48px; font-weight: bold;")
        title_layout.addWidget(self.cctv_label)
        title_layout.addWidget(self.robot_label)
        # 카메라 화면 레이아웃
        camera_layout = QHBoxLayout()
        self.cam1_label = QLabel()
        self.cam1_label.setMinimumSize(640, 480)
        self.cam1_label.setStyleSheet("border: 1px solid black;")
        self.cam1_label.setAlignment(Qt.AlignCenter)
        self.cam2_label = QLabel()
        self.cam2_label.setMinimumSize(640, 480)
        self.cam2_label.setStyleSheet("border: 1px solid black;")
        self.cam2_label.setAlignment(Qt.AlignCenter)
        camera_layout.addWidget(self.cam1_label)
        camera_layout.addWidget(self.cam2_label)
        # 버튼 및 상태 레이아웃
        control_layout = QHBoxLayout()
        self.run_button = QPushButton('Run')
        self.stop_button = QPushButton('Stop')
        self.run_button.clicked.connect(self.send_run_command)
        self.stop_button.clicked.connect(self.send_stop_command)
        self.status_label = QLabel('로봇 상태: READY')
        self.status_label.setAlignment(Qt.AlignCenter)
        control_layout.addWidget(self.run_button)
        control_layout.addWidget(self.stop_button)
        control_layout.addWidget(self.status_label)
        # 메인 레이아웃 조립
        main_layout.addLayout(title_layout)
        main_layout.addLayout(camera_layout)
        main_layout.addLayout(control_layout)
        self.show()
    def update_status_label(self, status_text):
        self.status_label.setText(f"로봇 상태: {status_text}")
    def send_run_command(self):
        self.ros_node.start_subscription()
    def send_stop_command(self):
        self.ros_node.stop_subscription()
        self.clear_images()
    def update_cam1(self, cv_image):
        self.set_image(self.cam1_label, cv_image)
    def update_cam2(self, cv_image):
        self.set_image(self.cam2_label, cv_image)
    def set_image(self, label, cv_image):
        height, width, channel = cv_image.shape
        bytes_per_line = 3 * width
        q_image = QImage(cv_image.data, width, height, bytes_per_line, QImage.Format_BGR888)
        label.setPixmap(QPixmap.fromImage(q_image))
    def clear_images(self):
        self.cam1_label.setPixmap(QPixmap())
        self.cam2_label.setPixmap(QPixmap())
    def closeEvent(self, event):
        self.ros_node.stop_subscription()
        event.accept()
if __name__ == '__main__':
    rclpy.init()
    app = QApplication(sys.argv)
    # Qt 브리지 객체 생성
    qt_bridge = ROS2Bridge()
    # ROS2 노드 생성
    ros_node = Server(qt_bridge)
    # GUI 생성
    window = MainWindow(ros_node)
    # 시그널 연결
    qt_bridge.cam1_signal.connect(window.update_cam1)
    qt_bridge.cam2_signal.connect(window.update_cam2)
    qt_bridge.status_signal.connect(window.update_status_label)
    # ROS2 스레드 시작
    ros_thread = ROSThread(ros_node)
    ros_thread.start()
    # GUI 실행
    exit_code = app.exec_()
    # 종료 처리
    ros_thread.stop()
    rclpy.shutdown()
    sys.exit(exit_code)