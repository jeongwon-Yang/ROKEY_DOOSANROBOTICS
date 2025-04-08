#!/usr/bin/env python3
import sys
import time
import json
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget,
    QLabel, QPushButton, QSpinBox, QTextEdit, QGridLayout
)
from PyQt5.QtGui import QImage, QPixmap

class MainROSNode(Node):
    def __init__(self):
        super().__init__('main_qt_conveyor_node')
        # YOLO 결과 영상 구독 (바운딩박스가 그려진 결과)
        self.sub_detected = self.create_subscription(
            CompressedImage,
            'yolo/compressed',
            self.yolo_image_callback,
            10
        )
        # 컨베이어 상태 구독
        self.sub_status = self.create_subscription(
            String,
            'conveyor/status',
            self.status_callback,
            10
        )
        # 컨베이어 제어 퍼블리셔
        self.pub_control = self.create_publisher(String, 'conveyor/control', 10)
        # 내부 데이터 변수
        self.image_np = None
        self.conveyor_status = "INIT"

    def yolo_image_callback(self, msg: CompressedImage):
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is not None:
            self.image_np = frame

    def status_callback(self, msg: String):
        self.conveyor_status = msg.data
        self.get_logger().info(f"[Conveyor Status] {self.conveyor_status}")

    def send_conveyor_command(self, command_dict):
        # command_dict 예: {"control": "go", "red_quantity": 1, "blue_quantity": 2} 또는 {"control": "stop"}
        data_str = json.dumps(command_dict)
        ros_msg = String()
        ros_msg.data = data_str
        self.pub_control.publish(ros_msg)
        self.get_logger().info(f"Sent conveyor command: {ros_msg.data}")

class MainWindow(QMainWindow):
    def __init__(self, ros_node: MainROSNode):
        super().__init__()
        self.ros_node = ros_node
        self.isClosed = False

        self.setWindowTitle("Conveyor + YOLO (Detected Image)")

        central = QWidget()
        self.setCentralWidget(central)
        self.layout = QGridLayout(central)

        # 좌측: 카메라 영상 표시 (640x480)
        self.labelCamera = QLabel("YOLO Detected Feed")
        self.labelCamera.setFixedSize(640, 480)
        self.layout.addWidget(self.labelCamera, 0, 0, 4, 1)

        # 좌측 하단: 간단 상태 표시
        self.labelStatus = QLabel("INIT")
        self.layout.addWidget(self.labelStatus, 4, 0, 1, 1)

        # 우측 상단: 빨간색 박스 개수 선택 (최대 2개)
        self.layout.addWidget(QLabel("Red Box Count (max 2):"), 0, 1)
        self.redSpinBox = QSpinBox()
        self.redSpinBox.setRange(0, 2)
        self.redSpinBox.setValue(0)
        self.layout.addWidget(self.redSpinBox, 1, 1)

        # 우측 중간: 파란색 박스 개수 선택 (최대 2개)
        self.layout.addWidget(QLabel("Blue Box Count (max 2):"), 2, 1)
        self.blueSpinBox = QSpinBox()
        self.blueSpinBox.setRange(0, 2)
        self.blueSpinBox.setValue(0)
        self.layout.addWidget(self.blueSpinBox, 3, 1)

        # 우측: GO 및 STOP 버튼
        self.btnGo = QPushButton("GO")
        self.btnGo.clicked.connect(self.on_go_clicked)
        self.layout.addWidget(self.btnGo, 4, 1)

        self.btnStop = QPushButton("STOP")
        self.btnStop.clicked.connect(self.on_stop_clicked)
        self.layout.addWidget(self.btnStop, 5, 1)

        # 우측 하단: 추가 INFO 창 (실시간 상태 메시지 출력)
        self.infoTextEdit = QTextEdit()
        self.infoTextEdit.setReadOnly(True)
        self.infoTextEdit.setPlaceholderText("INFO messages will appear here...")
        self.layout.addWidget(self.infoTextEdit, 6, 0, 1, 2)

        # 하단: Exit 버튼 (전체 너비 사용)
        self.btnExit = QPushButton("Exit")
        self.btnExit.clicked.connect(self.close)
        self.layout.addWidget(self.btnExit, 7, 0, 1, 2)

        self.show()

    def on_go_clicked(self):
        red_count = self.redSpinBox.value()
        blue_count = self.blueSpinBox.value()
        # 총 선택한 박스 개수가 0이면 경고 메시지 출력
        if red_count == 0 and blue_count == 0:
            self.infoTextEdit.append("No boxes selected!")
            return
        # GO 명령에 박스 종류별 개수를 포함하여 퍼블리시
        command_dict = {"control": "go", "red_quantity": red_count, "blue_quantity": blue_count}
        self.ros_node.send_conveyor_command(command_dict)
        self.infoTextEdit.append(f"GO command sent: Red: {red_count}, Blue: {blue_count}")

    def on_stop_clicked(self):
        command_dict = {"control": "stop"}
        self.ros_node.send_conveyor_command(command_dict)
        self.infoTextEdit.append("STOP command sent")

    def closeEvent(self, event):
        self.isClosed = True
        super().closeEvent(event)

    def update_gui(self):
        # 영상 업데이트 (BGR → RGB 변환)
        if self.ros_node.image_np is not None:
            frame_rgb = cv2.cvtColor(self.ros_node.image_np, cv2.COLOR_BGR2RGB)
            h, w, c = frame_rgb.shape
            qimg = QImage(frame_rgb.data, w, h, w * c, QImage.Format_RGB888)
            pixmap = QPixmap.fromImage(qimg)
            self.labelCamera.setPixmap(pixmap)
        # 상태 업데이트
        self.labelStatus.setText(self.ros_node.conveyor_status)

def main(args=None):
    rclpy.init(args=args)
    ros_node = MainROSNode()
    app = QApplication(sys.argv)
    window = MainWindow(ros_node)

    while not window.isClosed:
        rclpy.spin_once(ros_node, timeout_sec=0.01)
        window.update_gui()
        app.processEvents()
        time.sleep(0.01)

    ros_node.destroy_node()
    rclpy.shutdown()
    sys.exit(0)

if __name__ == '__main__':
    main()
