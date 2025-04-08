import os
import yaml
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Bool, Float32
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QPushButton, QLabel, QProgressBar, QTextEdit, QComboBox,
    QGroupBox, QTabWidget, QSlider, QFrame,
    QDialog, QSplitter
)
from PyQt5.QtCore import QThread, pyqtSignal, QTimer, Qt
from PyQt5.QtGui import QPixmap, QImage

import sys
import cv2
import numpy as np
from cv_bridge import CvBridge
import time
import threading

import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped

class ROS2Thread(QThread):
    """ROS2 백엔드 쓰레드: 배터리 감소, 카메라/오도메트리/매니퓰레이터 관절 등 처리"""
    # GUI로 보내는 시그널
    camera_signal = pyqtSignal(str, np.ndarray)  # (namespace, camera_img)
    odom_signal = pyqtSignal(str, float, float)  # (namespace, x, y)
    map_signal = pyqtSignal(np.ndarray)
    joint_state_signal = pyqtSignal(str, dict)   # (namespace, {joint_name: {pos,vel,eff}})
    # 인위적 배터리 감소를 위해 사용
    battery_signal = pyqtSignal(str, float)      # (namespace, battery_value)

    def __init__(self):
        super().__init__()
        rclpy.init()
        self.node = Node('integrated_gui')
        self.bridge = CvBridge()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.node)

        # ========== 맵 관련 ==========
        self.yaml_file = "/home/songhyun/ros2_ws/src/turtlebot3_multi_robot/maps/map.yaml"
        self.pgm_file = "/home/songhyun/ros2_ws/src/turtlebot3_multi_robot/maps/map.pgm"
        self.map_img = None
        self.resolution = 0.05
        self.origin_x = 0.0
        self.origin_y = 0.0
        self.width = 0
        self.height = 0

        # 로봇 네임스페이스 목록 (여기에 구조 로봇, 정찰 로봇, 매니퓰레이터 등 포함)
        self.robot_namespaces = ['tb1', 'tb2', 'manipulator']

        # 로봇 위치 관리 (odom)
        self.robot_poses = {}

        # ROS2 구독/게시자 보관
        self.subscribers = {}
        self.publishers = {}

        # ========== 배터리 관리 ==========
        # 인위적으로 배터리를 1초마다 1%씩 감소시킬 때 사용
        # 실제 배터리 토픽을 구독하는 대신, 여기서 강제로 줄임
        # (원하면 실제 토픽 구독 로직도 병행할 수 있음)
        self.battery_levels = {
            'tb1': 100.0,   # 정찰 로봇
            'tb2': 100.0,   # 구조 로봇
            'manipulator': 100.0
        }
        self.battery_threshold = 10.0  # 10% 임계치
        self.battery_timer = QTimer()
        self.battery_timer.timeout.connect(self.reduce_battery)
        self.battery_timer.start(1000)  # 1초마다 1%씩 감소

        # rescue 로직 중복 실행 방지용 플래그
        self.rescue_triggered = False

        # ========== 맵 로드 ==========
        self.load_map()

        # ========== 각 로봇별 구독/게시자 설정 ==========
        for ns in self.robot_namespaces:
            if ns in ['tb1', 'tb2']:
                # 오도메트리 구독
                self.subscribers[f'{ns}_odom'] = self.node.create_subscription(
                    Odometry, f'/{ns}/odom',
                    lambda msg, ns=ns: self.odom_callback(msg, ns), 10)

                # 카메라 구독
                self.subscribers[f'{ns}_camera'] = self.node.create_subscription(
                    Image, f'/{ns}/camera/image_raw',
                    lambda msg, ns=ns: self.camera_callback(msg, ns), 10)

                # 이동 명령 (cmd_vel) 게시자
                self.publishers[f'{ns}_cmd_vel'] = self.node.create_publisher(
                    Twist, f'/{ns}/cmd_vel', 10)

                # 네비게이션 목표(Goal) 게시자
                self.publishers[f'{ns}_goal'] = self.node.create_publisher(
                    PoseStamped, f'/{ns}/goal_pose', 10)

            if ns == 'manipulator':
                # 매니퓰레이터 joint_state 구독
                self.subscribers['manipulator_joint_states'] = self.node.create_subscription(
                    JointState, '/manipulator/joint_states',
                    lambda msg: self.joint_state_callback(msg, 'manipulator'), 10)

                # 매니퓰레이터 arm, gripper 게시자
                self.publishers['manipulator_arm'] = self.node.create_publisher(
                    JointTrajectory, '/manipulator/arm_controller/commands', 10)
                self.publishers['manipulator_gripper'] = self.node.create_publisher(
                    JointTrajectory, '/manipulator/gripper_controller/commands', 10)

        # ROS spin 별도 스레드
        self.ros_spin_thread = threading.Thread(target=self.ros_spin, daemon=True)
        self.ros_spin_thread.start()

    def ros_spin(self):
        while rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.1)
    def transform_odom_to_map(self, namespace: str):
        if namespace not in self.robot_poses:
            self.node.get_logger().warn(f"{namespace}의 위치 정보가 없습니다.")
            return None

        pose = self.robot_poses[namespace]
        odom_pose = PoseStamped()
        odom_pose.header.frame_id = 'odom'
        odom_pose.header.stamp = self.node.get_clock().now().to_msg()
        odom_pose.pose.position.x = pose['x']
        odom_pose.pose.position.y = pose['y']
        odom_pose.pose.position.z = 0.0
        # 기본 orientation (회전 없음)
        odom_pose.pose.orientation.x = 0.0
        odom_pose.pose.orientation.y = 0.0
        odom_pose.pose.orientation.z = 0.0
        odom_pose.pose.orientation.w = 1.0

        try:
            transform = self.tf_buffer.lookup_transform(
                'map',  # 목표 프레임
                'odom', # 원본 프레임
                rclpy.time.Time()
            )
            map_pose = tf2_geometry_msgs.do_transform_pose(odom_pose, transform)
            return map_pose
        except Exception as e:
            self.node.get_logger().error(f"[{namespace}] odom->map 변환 실패: {e}")
            return None

    # ================== 맵 로드 & 표시 ===================
    def load_map(self):
        if not os.path.exists(self.yaml_file) or not os.path.exists(self.pgm_file):
            self.node.get_logger().error("맵 파일을 찾을 수 없습니다!")
            return

        try:
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
            map_data = img_gray.astype(np.uint8)

            # 맵 색상 변환
            map_data = np.where(map_data == 205, 127, map_data)  # 미탐색 → 회색
            map_data = np.where(map_data == 254, 255, map_data)  # 빈 공간 → 흰색
            map_data = np.where(map_data == 0, 0, map_data)      # 장애물 → 검정색

            # 컬러 맵 생성
            self.map_img = np.zeros((self.height, self.width, 3), dtype=np.uint8)
            for i in range(self.height):
                for j in range(self.width):
                    val = map_data[i, j]
                    if val == 0:
                        self.map_img[i, j] = (0, 0, 0)      # 장애물 - 검정색
                    elif val == 127:
                        self.map_img[i, j] = (127, 127, 127)  # 미탐색 - 회색
                    elif val == 255:
                        self.map_img[i, j] = (255, 255, 255)  # 빈 공간 - 흰색

            self.node.get_logger().info(
                f"맵 로드 완료: 크기=({self.width}x{self.height}), "
                f"해상도={self.resolution}, 원점=({self.origin_x:.2f}, {self.origin_y:.2f})"
            )

            # 초기 맵 이미지를 한 번 UI로 전송
            self.map_signal.emit(self.map_img.copy())

        except Exception as e:
            self.node.get_logger().error(f"맵 로드 중 오류: {e}")

    # ================== 콜백들 ===================
    def odom_callback(self, msg: Odometry, namespace: str):
        # 전체 Pose를 복사해서 사용
        adjusted_pose = msg.pose.pose
        # 오프셋 적용: x +2, y +0.5
        adjusted_pose.position.x += 2.0
        adjusted_pose.position.y += 0.5
        # 전체 Pose를 robot_poses에 저장 (이제는 Pose 객체)
        self.robot_poses[namespace] = adjusted_pose
        # UI 업데이트용 signal에는 오프셋 적용된 좌표를 전달
        self.odom_signal.emit(namespace, adjusted_pose.position.x, adjusted_pose.position.y)


    def camera_callback(self, msg: Image, namespace: str):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            if img is not None:
                self.camera_signal.emit(namespace, img)
        except Exception as e:
            self.node.get_logger().error(f"카메라 이미지 처리 오류 [{namespace}]: {e}")

    def joint_state_callback(self, msg: JointState, namespace: str):
        joint_state = {}
        for i, name in enumerate(msg.name):
            pos = msg.position[i] if i < len(msg.position) else 0.0
            vel = msg.velocity[i] if i < len(msg.velocity) else 0.0
            eff = msg.effort[i]   if i < len(msg.effort)   else 0.0
            joint_state[name] = {'position': pos, 'velocity': vel, 'effort': eff}

        self.joint_state_signal.emit(namespace, joint_state)

    # ================== 배터리 감소 로직. ===================
    def reduce_battery(self):
        for ns in self.battery_levels:
            if self.battery_levels[ns] > 0:
                self.battery_levels[ns] -= 1
                if self.battery_levels[ns] < 0:
                    self.battery_levels[ns] = 0
            self.battery_signal.emit(ns, self.battery_levels[ns])

        if not self.rescue_triggered and self.battery_levels['tb1'] <= self.battery_threshold:
            self.rescue_triggered = True
            # tb1 정지
            self.send_cmd_vel('tb1', 0.0, 0.0)
            # tb1의 저장된 전체 Pose를 사용
            current_pose = self.robot_poses.get('tb1', None)
            if current_pose is not None:
                self.node.get_logger().warn(f"[tb1] 배터리 부족! 좌표: x={current_pose.position.x:.2f}, y={current_pose.position.y:.2f}")
                # 원하는 경우, 이 좌표를 GUI 로그에 출력한 후 사용자가 직접 목표 입력란에 복사해서 tb2로 전송하거나,
                # 자동 전송을 할 수도 있습니다.
                # 예를 들어 자동으로 목표를 전송하려면 아래와 같이 할 수 있습니다.
                goal_msg = PoseStamped()
                goal_msg.header.stamp = self.node.get_clock().now().to_msg()
                goal_msg.header.frame_id = 'map'
                goal_msg.pose = current_pose  # 바로 사용
                # 목표 게시자: tb2의 goal 게시자
                if 'tb2_goal' in self.publishers:
                    self.publishers['tb2_goal'].publish(goal_msg)
                    self.node.get_logger().info(f"[tb2] 구조 목표 전송: x={current_pose.position.x:.2f}, y={current_pose.position.y:.2f}")
                else:
                    self.node.get_logger().error("tb2의 goal 게시자가 없습니다.")
            else:
                self.node.get_logger().warn("[tb1] 현재 위치 정보가 없습니다.")


    # ================== 퍼블리시 유틸 함수 ===================
    def send_cmd_vel(self, namespace: str, linear_x=0.0, angular_z=0.0):
        pub_key = f'{namespace}_cmd_vel'
        if pub_key not in self.publishers:
            self.node.get_logger().error(f"로봇 {namespace}의 cmd_vel 게시자가 없습니다.")
            return
        twist = Twist()
        twist.linear.x = float(linear_x)
        twist.angular.z = float(angular_z)
        self.publishers[pub_key].publish(twist)

    def send_nav_goal(self, namespace: str, x: float, y: float):
        """네비게이션 목표"""
        pub_key = f'{namespace}_goal'
        if pub_key not in self.publishers:
            self.node.get_logger().error(f"로봇 {namespace}의 goal_pose 게시자가 없습니다.")
            return
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = 'map'
        goal_msg.header.stamp = self.node.get_clock().now().to_msg()
        goal_msg.pose.position.x = x
        goal_msg.pose.position.y = y
        goal_msg.pose.orientation.w = 1.0
        self.publishers[pub_key].publish(goal_msg)
        self.node.get_logger().info(f"[{namespace}] Goal set to x={x:.2f}, y={y:.2f}")

    def send_arm_command(self, positions):
        """매니퓰레이터 관절 제어"""
        if 'manipulator_arm' not in self.publishers:
            self.node.get_logger().error("매니퓰레이션 로봇 암 게시자가 없습니다.")
            return
        traj = JointTrajectory()
        traj.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(sec=2, nanosec=0)
        traj.points = [point]
        self.publishers['manipulator_arm'].publish(traj)

    def send_gripper_command(self, position):
        """매니퓰레이션 그리퍼 제어"""
        if 'manipulator_gripper' not in self.publishers:
            self.node.get_logger().error("매니퓰레이션 그리퍼 게시자가 없습니다.")
            return
        traj = JointTrajectory()
        traj.joint_names = ['gripper']
        point = JointTrajectoryPoint()
        point.positions = [position]
        point.time_from_start = Duration(sec=1, nanosec=0)
        traj.points = [point]
        self.publishers['manipulator_gripper'].publish(traj)

    # ================== 맵 주기적 업데이트 (로봇 위치 표시) ===================
    def update_visualization(self):
        if self.map_img is None:
            self.node.get_logger().error("맵 이미지가 로드되지 않았습니다!")
            return

        vis_img = self.map_img.copy()

        for ns, pose in self.robot_poses.items():
            # pose가 dict일 경우와 Pose 객체일 경우 모두 처리
            if isinstance(pose, dict):
                x = pose['x']
                y = pose['y']
            else:
                x = pose.position.x
                y = pose.position.y

            pixel_x = int((x - self.origin_x) / self.resolution)
            pixel_y = int((y - self.origin_y) / self.resolution)
            pixel_y = self.height - pixel_y  # OpenCV 기준 y축 반전

            if 0 <= pixel_x < self.width and 0 <= pixel_y < self.height:
                cv2.circle(vis_img, (pixel_x, pixel_y), 5, (0, 255, 0), -1)
                cv2.putText(vis_img, ns, (pixel_x+8, pixel_y-8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        self.map_signal.emit(vis_img)

class IntegratedGUI(QMainWindow):
    """탭 구조 + 맵/카메라/로그를 통합한 관제 GUI"""
    def __init__(self):
        super().__init__()
        self.setWindowTitle("통합 로봇 관제 시스템")
        self.setGeometry(100, 100, 1400, 800)

        # 중앙 위젯
        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)

        # 메인 레이아웃 (수평)
        self.main_layout = QHBoxLayout(self.central_widget)

        # 왼쪽: 로봇 제어 탭
        self.left_panel = QTabWidget()
        self.main_layout.addWidget(self.left_panel, 1)

        # 오른쪽: 맵/카메라/로그
        self.right_panel = QVBoxLayout()
        self.right_frame = QFrame()
        self.right_frame.setLayout(self.right_panel)
        self.main_layout.addWidget(self.right_frame, 2)

        # ROS2 스레드
        self.ros2_thread = ROS2Thread()
        self.ros2_thread.start()

        # 각 로봇 탭 생성
        self.setup_robot_tabs()

        # 오른쪽: 맵 + 카메라 + 로그
        self.setup_map_and_camera()
        self.setup_log()

        # 시그널 연결
        self.ros2_thread.battery_signal.connect(self.update_battery)
        self.ros2_thread.camera_signal.connect(self.update_camera)
        self.ros2_thread.odom_signal.connect(self.update_odom)
        self.ros2_thread.map_signal.connect(self.update_map)
        self.ros2_thread.joint_state_signal.connect(self.update_joint_state)

        # 주기적 맵 업데이트
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.ros2_thread.update_visualization)
        self.timer.start(100)  # 10Hz

        # 현재 선택된 로봇(카메라 표시용)
        self.current_robot = 'tb1'

    # -----------------------------
    # 로봇 탭 구성
    # -----------------------------
    def setup_robot_tabs(self):
        """tb1, tb2, manipulator 탭 생성"""
        # tb1 탭
        self.tb1_tab = QWidget()
        self.tb1_layout = QVBoxLayout(self.tb1_tab)
        self.left_panel.addTab(self.tb1_tab, "TurtleBot #1")
        self.setup_turtlebot_controls(self.tb1_tab, self.tb1_layout, 'tb1')

        # tb2 탭
        self.tb2_tab = QWidget()
        self.tb2_layout = QVBoxLayout(self.tb2_tab)
        self.left_panel.addTab(self.tb2_tab, "TurtleBot #2")
        self.setup_turtlebot_controls(self.tb2_tab, self.tb2_layout, 'tb2')

        # manipulator 탭
        self.manip_tab = QWidget()
        self.manip_layout = QVBoxLayout(self.manip_tab)
        self.left_panel.addTab(self.manip_tab, "매니퓰레이터")
        self.setup_manipulation_controls(self.manip_tab, self.manip_layout)

    def setup_turtlebot_controls(self, tab, layout, namespace):
        """터틀봇 제어 UI"""
        # 배터리
        battery_group = QGroupBox("배터리 상태")
        battery_layout = QVBoxLayout()
        self.battery_bars = getattr(self, 'battery_bars', {})
        self.battery_bars[namespace] = QProgressBar()
        self.battery_bars[namespace].setRange(0, 100)
        self.battery_bars[namespace].setValue(100)
        battery_layout.addWidget(self.battery_bars[namespace])
        battery_group.setLayout(battery_layout)
        layout.addWidget(battery_group)

        # 위치
        position_group = QGroupBox("위치 정보")
        position_layout = QVBoxLayout()
        self.position_labels = getattr(self, 'position_labels', {})
        self.position_labels[namespace] = QLabel("X: 0.00, Y: 0.00")
        position_layout.addWidget(self.position_labels[namespace])
        position_group.setLayout(position_layout)
        layout.addWidget(position_group)

        # 이동 제어
        control_group = QGroupBox("이동 제어")
        control_layout = QGridLayout()
        forward_btn = QPushButton("⬆ 전진")
        backward_btn = QPushButton("⬇ 후진")
        left_btn = QPushButton("⬅ 좌회전")
        right_btn = QPushButton("➡ 우회전")
        stop_btn = QPushButton("■ 정지")
        control_layout.addWidget(forward_btn, 0, 1)
        control_layout.addWidget(backward_btn, 2, 1)
        control_layout.addWidget(left_btn, 1, 0)
        control_layout.addWidget(right_btn, 1, 2)
        control_layout.addWidget(stop_btn, 1, 1)
        control_group.setLayout(control_layout)
        layout.addWidget(control_group)

        # 버튼 이벤트
        forward_btn.clicked.connect(lambda: self.ros2_thread.send_cmd_vel(namespace, 0.2, 0.0))
        backward_btn.clicked.connect(lambda: self.ros2_thread.send_cmd_vel(namespace, -0.2, 0.0))
        left_btn.clicked.connect(lambda: self.ros2_thread.send_cmd_vel(namespace, 0.0, 0.5))
        right_btn.clicked.connect(lambda: self.ros2_thread.send_cmd_vel(namespace, 0.0, -0.5))
        stop_btn.clicked.connect(lambda: self.ros2_thread.send_cmd_vel(namespace, 0.0, 0.0))

        # 목표점 이동
        nav_group = QGroupBox("목표점 이동")
        nav_layout = QVBoxLayout()
        x_layout = QHBoxLayout()
        x_layout.addWidget(QLabel("X 좌표:"))
        x_input = QTextEdit()
        x_input.setMaximumHeight(30)
        x_layout.addWidget(x_input)
        nav_layout.addLayout(x_layout)

        y_layout = QHBoxLayout()
        y_layout.addWidget(QLabel("Y 좌표:"))
        y_input = QTextEdit()
        y_input.setMaximumHeight(30)
        y_layout.addWidget(y_input)
        nav_layout.addLayout(y_layout)

        send_goal_btn = QPushButton("목표 전송")
        send_goal_btn.clicked.connect(lambda: self.send_nav_goal(namespace, x_input, y_input))
        nav_layout.addWidget(send_goal_btn)
        nav_group.setLayout(nav_layout)
        layout.addWidget(nav_group)

        # 카메라 선택
        cam_group = QGroupBox("카메라 선택")
        cam_layout = QVBoxLayout()
        cam_btn = QPushButton("이 로봇 카메라 보기")
        cam_btn.clicked.connect(lambda: self.set_current_robot(namespace))
        cam_layout.addWidget(cam_btn)
        cam_group.setLayout(cam_layout)
        layout.addWidget(cam_group)

    def setup_manipulation_controls(self, tab, layout):
        """매니퓰레이터 제어 UI"""
        namespace = 'manipulator'

        # 배터리
        battery_group = QGroupBox("배터리 상태")
        battery_layout = QVBoxLayout()
        self.battery_bars = getattr(self, 'battery_bars', {})
        self.battery_bars[namespace] = QProgressBar()
        self.battery_bars[namespace].setRange(0, 100)
        self.battery_bars[namespace].setValue(100)
        battery_layout.addWidget(self.battery_bars[namespace])
        battery_group.setLayout(battery_layout)
        layout.addWidget(battery_group)

        # 관절 제어
        joint_group = QGroupBox("관절 제어")
        joint_layout = QVBoxLayout()

        self.joint_sliders = {}
        self.joint_labels = getattr(self, 'joint_labels', {})
        for joint_name in ['joint1', 'joint2', 'joint3', 'joint4']:
            row_layout = QHBoxLayout()
            row_layout.addWidget(QLabel(joint_name))
            slider = QSlider(Qt.Horizontal)
            slider.setRange(-314, 314)  # -π~π 라디안*100
            label = QLabel("0.00")
            self.joint_sliders[joint_name] = slider
            self.joint_labels[joint_name] = label

            # 슬라이더 값 변경 시 라벨 업데이트
            slider.valueChanged.connect(
                lambda val, j=joint_name: self.joint_labels[j].setText(f"{val/100:.2f}")
            )

            row_layout.addWidget(slider)
            row_layout.addWidget(label)
            joint_layout.addLayout(row_layout)

        send_joints_btn = QPushButton("관절 위치 전송")
        send_joints_btn.clicked.connect(self.send_joint_command)
        joint_layout.addWidget(send_joints_btn)
        joint_group.setLayout(joint_layout)
        layout.addWidget(joint_group)

        # 그리퍼 제어
        gripper_group = QGroupBox("그리퍼 제어")
        gripper_layout = QHBoxLayout()
        open_btn = QPushButton("열기")
        close_btn = QPushButton("닫기")
        open_btn.clicked.connect(lambda: self.ros2_thread.send_gripper_command(0.01))
        close_btn.clicked.connect(lambda: self.ros2_thread.send_gripper_command(0.0))
        gripper_layout.addWidget(open_btn)
        gripper_layout.addWidget(close_btn)
        gripper_group.setLayout(gripper_layout)
        layout.addWidget(gripper_group)

        # 미리 정의된 포즈
        pose_group = QGroupBox("미리 정의된 포즈")
        pose_layout = QVBoxLayout()
        home_pose_btn = QPushButton("홈 포즈")
        home_pose_btn.clicked.connect(lambda: self.ros2_thread.send_arm_command([0.0, 0.0, 0.0, 0.0]))
        ready_pose_btn = QPushButton("준비 포즈")
        ready_pose_btn.clicked.connect(lambda: self.ros2_thread.send_arm_command([0.0, 0.5, 0.0, 0.0]))
        pick_pose_btn = QPushButton("집기 포즈")
        pick_pose_btn.clicked.connect(lambda: self.ros2_thread.send_arm_command([0.0, 0.7, -0.3, 0.0]))
        pose_layout.addWidget(home_pose_btn)
        pose_layout.addWidget(ready_pose_btn)
        pose_layout.addWidget(pick_pose_btn)
        pose_group.setLayout(pose_layout)
        layout.addWidget(pose_group)

    # -----------------------------
    # 오른쪽 패널: 맵 & 카메라 & 로그
    # -----------------------------
    def setup_map_and_camera(self):
        # 맵
        map_group = QGroupBox("맵 & 로봇 위치")
        map_layout = QVBoxLayout()
        self.map_label = QLabel("맵을 로드 중입니다...")
        self.map_label.setMinimumSize(600, 400)
        map_layout.addWidget(self.map_label)
        map_group.setLayout(map_layout)
        self.right_panel.addWidget(map_group)

        # 카메라
        cam_group = QGroupBox("카메라 뷰")
        cam_layout = QVBoxLayout()
        self.camera_selector = QComboBox()
        self.camera_selector.addItems(['tb1', 'tb2'])
        self.camera_selector.currentTextChanged.connect(self.set_current_robot)
        cam_layout.addWidget(self.camera_selector)

        self.camera_label = QLabel("카메라 연결 중...")
        self.camera_label.setMinimumSize(400, 300)
        cam_layout.addWidget(self.camera_label)

        cam_group.setLayout(cam_layout)
        self.right_panel.addWidget(cam_group)

    def setup_log(self):
        log_group = QGroupBox("시스템 로그")
        log_layout = QVBoxLayout()
        self.log_box = QTextEdit()
        self.log_box.setReadOnly(True)
        log_layout.addWidget(self.log_box)
        log_group.setLayout(log_layout)
        self.right_panel.addWidget(log_group)

    # -----------------------------
    # 시그널 처리 함수
    # -----------------------------
    def update_battery(self, namespace, value):
        if hasattr(self, 'battery_bars') and namespace in self.battery_bars:
            self.battery_bars[namespace].setValue(int(value))
            self.log(f"[{namespace}] 배터리: {value}%")

    def update_camera(self, namespace, img):
        """현재 선택된 로봇의 카메라만 표시"""
        if namespace == self.current_robot:
            h, w, c = img.shape
            bytes_per_line = 3 * w
            qimg = QImage(img.data, w, h, bytes_per_line, QImage.Format_RGB888)
            pixmap = QPixmap.fromImage(qimg)
            self.camera_label.setPixmap(pixmap)
            self.camera_label.setScaledContents(True)

    def update_odom(self, namespace, x, y):
        """위치 라벨 갱신"""
        if hasattr(self, 'position_labels') and namespace in self.position_labels:
            self.position_labels[namespace].setText(f"X: {x:.2f}, Y: {y:.2f}")

    def update_map(self, img):
        if img is None or img.size == 0:
            return
        h, w, c = img.shape
        bytes_per_line = 3 * w
        qimg = QImage(img.data, w, h, bytes_per_line, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(qimg)
        self.map_label.setPixmap(pixmap)
        self.map_label.setScaledContents(True)

    def update_joint_state(self, namespace, joint_state):
        """매니퓰레이터 관절 상태 갱신 -> 슬라이더/라벨 반영"""
        if namespace != 'manipulator':
            return
        if not hasattr(self, 'joint_sliders'):
            return

        for j_name, j_info in joint_state.items():
            if j_name in self.joint_sliders:
                pos = j_info['position']
                slider_val = int(pos * 100)
                self.joint_sliders[j_name].blockSignals(True)
                self.joint_sliders[j_name].setValue(slider_val)
                self.joint_sliders[j_name].blockSignals(False)
                if j_name in self.joint_labels:
                    self.joint_labels[j_name].setText(f"{pos:.2f}")

    # -----------------------------
    # 유틸
    # -----------------------------
    def send_nav_goal(self, namespace, x_input, y_input):
        try:
            x = float(x_input.toPlainText())
            y = float(y_input.toPlainText())
            self.ros2_thread.send_nav_goal(namespace, x, y)
            self.log(f"[{namespace}] Goal -> x={x:.2f}, y={y:.2f}")
        except ValueError:
            self.log("오류: 유효한 좌표를 입력하세요.")

    def set_current_robot(self, namespace):
        """카메라 표시 대상 로봇 변경"""
        self.current_robot = namespace
        if self.camera_selector.currentText() != namespace:
            self.camera_selector.setCurrentText(namespace)
        self.log(f"현재 로봇 카메라: {namespace}")

    def send_joint_command(self):
        """매니퓰레이터 관절 슬라이더 -> 명령 전송"""
        positions = []
        for j_name in ['joint1', 'joint2', 'joint3', 'joint4']:
            if j_name in self.ros2_thread.joint_sliders:
                val = self.ros2_thread.joint_sliders[j_name].value() / 100.0
                positions.append(val)
            else:
                # GUI 쪽 슬라이더를 사용
                val = self.joint_sliders[j_name].value() / 100.0
                positions.append(val)
        self.ros2_thread.send_arm_command(positions)
        self.log(f"[manipulator] 관절 명령 전송: {positions}")

    def log(self, msg):
        """로그 출력"""
        t = time.strftime("%H:%M:%S")
        self.log_box.append(f"[{t}] {msg}")
        self.log_box.ensureCursorVisible()

def main():
    app = QApplication(sys.argv)
    gui = IntegratedGUI()
    gui.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
