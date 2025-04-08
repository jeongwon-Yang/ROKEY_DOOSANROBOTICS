#!/usr/bin/env python3
# kitchen_ex.py

import sys
import signal
import threading
import json
from functools import partial
from datetime import datetime
import time  # weight 감소 시 단순 sleep 예시

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# nav2 NavigateToPose 액션 관련
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from rclpy.action.client import GoalStatus

# PyQt5 기준
from PyQt5.QtCore import Qt, pyqtSignal, QTimer
from PyQt5.QtWidgets import (
    QApplication, QWidget, QLabel, QPushButton, QVBoxLayout, QHBoxLayout,
    QFrame, QGridLayout
)

class KitchenApp(Node, QWidget):
    # Node → GUI로 주문 정보를 전달하기 위한 시그널
    # 파라미터 (table_id: int, menu_items: list, is_takeout: str)
    order_signal = pyqtSignal(int, list, str)

    def __init__(self):
        Node.__init__(self, node_name='kitchen_app')
        QWidget.__init__(self)

        self.setWindowTitle("Kitchen App")
        self.resize(800, 600)

        # 시그널 연결: 콜백에서 emit → 이 슬롯에서 UI 갱신
        self.order_signal.connect(self.update_table_ui)

        # 테이블 정보
        self.tables = []
        self.table_count = 9

        # 재고 관련 퍼블리셔 (필요시 사용)
        self.publisher_ = self.create_publisher(String, 'inventory_warning', 10)

        # 서브스크라이버: order_topic
        self.subscription = self.create_subscription(
            String,
            'order_topic',
            self.subscription_callback,
            10
        )

        # ---- (추가) Nav2 액션 클라이언트 준비 ----
        # nav2가 /navigate_to_pose라는 액션 서버를 구동 중이라고 가정
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # 초기 위치(주방) 좌표 (질문에서 주신 /set_initial_pose와 동일)
        self.home_pose = (-2.008, -0.54109, 0.0, 1.0)

        # 테이블별 (x, y) 좌표 (질문에서 주신 값들을 사용)
        # 모두 z=0, w=1.0 (방향 0도) 로 가정. 필요시 각 테이블의 orientation 조정.
        self.table_coords = [
            ( 0.912933886051178,  0.7711067199707031, 0.0, 1.0),  # 1번 테이블
            ( 0.9351988434791565, -0.2846567630767822, 0.0, 1.0), # 2번
            ( 0.9482356309890747, -1.3495535850524902, 0.0, 1.0), # 3번
            (-0.22553983330726624,  0.8043422102928162, 0.0, 1.0),# 4번
            (-0.21291649341583252, -0.34316912293434143, 0.0, 1.0),#5번
            (-0.2094295620918274,  -1.4814550876617432, 0.0, 1.0),# 6번
            (-1.3182857036590576,  0.8006305694580078, 0.0, 1.0), # 7번
            (-1.3239352703094482, -0.32843017578125,   0.0, 1.0), # 8번
            (-1.311036467552185,  -1.4208654165267944, 0.0, 1.0), # 9번
        ]

        # 메뉴 재고
        self.menu_inventory = {
            "짜장면": 10, "간짜장": 10, "짬뽕": 10, "해물짬뽕": 10, "볶음밥": 10, "탕수육": 10,
            "잡채밥": 10, "잡탕밥": 10, "군만두": 10, "찐만두": 10, "콜라": 10, "사이다": 10, "제로콜라": 10
        }
        self.inventory_visible = False

        # 레이아웃 설정
        self.layout = QHBoxLayout()
        self.left_layout = QVBoxLayout()
        self.right_layout = QVBoxLayout()

        self.table_layout = QGridLayout()
        self.left_layout.addLayout(self.table_layout)

        # 인벤토리 버튼
        self.inventory_button = QPushButton("Inventory", self)
        self.inventory_button.clicked.connect(self.toggle_inventory)
        self.left_layout.addWidget(self.inventory_button)

        # 인벤토리 영역 (기본적으로 숨김)
        self.inventory_frame = QFrame(self)
        self.inventory_frame.setFrameShape(QFrame.Box)
        self.inventory_frame.setStyleSheet("background-color: lightgray;")
        self.inventory_frame.setVisible(False)
        self.left_layout.addWidget(self.inventory_frame)

        self.layout.addLayout(self.left_layout)
        self.layout.addLayout(self.right_layout)
        self.setLayout(self.layout)

        # 인벤토리 레이아웃(예: 추가 정보)
        inventory_layout = QVBoxLayout(self.inventory_frame)
        self.inventory_labels = []
        for item, count in self.menu_inventory.items():
            label = QLabel(f"{item}: {count}개", self.inventory_frame)
            inventory_layout.addWidget(label)
            self.inventory_labels.append((item, label))

        # 테이블들 생성
        self.create_tables()

    def create_tables(self):
        for i in range(self.table_count):
            table_widget = QFrame(self)
            table_widget.setFrameShape(QFrame.Box)
            table_layout = QGridLayout(table_widget)

            # 테이블 번호
            table_number_label = QLabel(f"{i + 1}", self)
            table_number_label.setAlignment(Qt.AlignLeft | Qt.AlignTop)
            table_number_label.setStyleSheet("font-size: 10px;")

            # 주문 번호(또는 테이블명) 표시
            order_number_label = QLabel("", self)
            order_number_label.setAlignment(Qt.AlignHCenter | Qt.AlignTop)
            order_number_label.setStyleSheet("font-size: 10px;")

            # 시간 표시
            time_label = QLabel("", self)
            time_label.setAlignment(Qt.AlignRight | Qt.AlignTop)
            time_label.setStyleSheet("font-size: 10px;")

            # 포장 여부 표시
            packing_label = QLabel("", self)
            packing_label.setAlignment(Qt.AlignRight | Qt.AlignBottom)
            packing_label.setStyleSheet("font-size: 10px;")

            # 주문 내역
            order_details_label = QLabel("주문 내역: 없음", self)
            order_details_label.setWordWrap(True)
            order_details_label.setAlignment(Qt.AlignCenter)

            # Complete 버튼: 여기서 로봇을 테이블로 이동
            complete_button = QPushButton("Complete", self)
            complete_button.clicked.connect(partial(self.complete_table, i))

            # Clear 버튼
            clear_button = QPushButton("Clear", self)
            clear_button.clicked.connect(partial(self.clear_table, i))

            table_layout.addWidget(table_number_label,    0, 0)
            table_layout.addWidget(order_number_label,    0, 1)
            table_layout.addWidget(time_label,            0, 2)
            table_layout.addWidget(order_details_label,   1, 0, 1, 3)
            table_layout.addWidget(packing_label,         2, 2)
            table_layout.addWidget(complete_button,       3, 0)
            table_layout.addWidget(clear_button,          3, 2)

            self.tables.append({
                "widget": table_widget,
                "order_details": order_details_label,
                "time": time_label,
                "packing": packing_label,
                "order_number": order_number_label,
            })

            self.table_layout.addWidget(table_widget, i // 3, i % 3)

    # -------------------------
    # 인벤토리 열고/닫기
    # -------------------------
    def toggle_inventory(self):
        self.inventory_visible = not self.inventory_visible
        self.inventory_frame.setVisible(self.inventory_visible)

    # -------------------------
    # ROS 콜백 (별도 스레드)
    # -------------------------
    def subscription_callback(self, msg: String):
        self.get_logger().info(f"Received raw message: {msg.data}")
        try:
            order_data = json.loads(msg.data)
            table_str = order_data.get("table_number", "테이블 1")
            menu_items = order_data.get("menu_items", [])
            is_takeout_str = "포장" if order_data.get("is_takeout", False) else "매장"

            # 테이블 번호 파싱
            table_id = 0
            if "테이블" in table_str:
                # "테이블 3" 형태
                num_str = table_str.split()[-1]  # 3
                table_id = int(num_str) - 1

            # 여기서 직접 UI를 건드리지 않고, 시그널 emit
            self.order_signal.emit(table_id, menu_items, is_takeout_str)

        except Exception as e:
            self.get_logger().error(f"Failed to parse order data: {e}")

    # -------------------------
    # 시그널 연결된 슬롯(메인 스레드)
    # -------------------------
    def update_table_ui(self, table_id, menu_items, is_takeout):
        if table_id < 0 or table_id >= len(self.tables):
            table_id = 0

        table = self.tables[table_id]
        menu_summary = "\n".join([f"{item['name']} x{item['quantity']}" for item in menu_items])
        table["order_details"].setText(f"주문 내역:\n{menu_summary}")
        table["time"].setText(datetime.now().strftime("%H:%M:%S"))
        table["packing"].setText(is_takeout)
        table["order_number"].setText(f"Table {table_id + 1}")

    # -------------------------
    # Complete 버튼 로직 확장
    # -------------------------
    def complete_table(self, index):
        # 1) 색상 변경 (기존 로직)
        self.tables[index]["widget"].setStyleSheet("background-color: lightgreen;")

        # 2) 로봇 이동 명령 (Nav2 액션 클라이언트)
        self.get_logger().info(f"[COMPLETE] Sending robot to table {index+1}")

        # 테이블 좌표 가져오기
        if index < len(self.table_coords):
            tx, ty, tz, tw = self.table_coords[index]
        else:
            self.get_logger().warn("Index out of table_coords range. Using default=table1.")
            tx, ty, tz, tw = self.table_coords[0]

        # Goal 전송 (비동기)
        self.send_goal_to_nav2(tx, ty, tz, tw, callback_when_done=self.after_reaching_table)

    def after_reaching_table(self, status):
        """ 테이블 도착 후 weight 감소 로직 & 복귀 명령 """
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("[INFO] Robot arrived at table. Simulating weight drop...")

            # TODO: 여기서 weight를 설정하고, 몇 초간 대기하면서 감소시킨 뒤 복귀
            #      실제로는 타이머를 사용하거나, 실시간으로 줄여가는 로직을 넣을 수 있음
            # 여기서는 예시로 5초 기다린다고 가정
            time.sleep(5.0)

            self.get_logger().info("[INFO] Weight = 0. Returning to Home Position...")

            # 주방(초기 위치)로 복귀
            hx, hy, hz, hw = self.home_pose
            self.send_goal_to_nav2(hx, hy, hz, hw, callback_when_done=self.after_return_home)

        else:
            self.get_logger().warn(f"[WARN] Robot failed to arrive at table. status={status}")

    def after_return_home(self, status):
        """ 홈(주방) 복귀 후 처리 """
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("[INFO] Robot successfully returned home!")
        else:
            self.get_logger().warn("[WARN] Robot failed to return home.")

    # -------------------------
    # Clear 버튼
    # -------------------------
    def clear_table(self, index):
        self.tables[index]["widget"].setStyleSheet("")
        self.tables[index]["order_details"].setText("주문 내역: 없음")
        self.tables[index]["time"].setText("")
        self.tables[index]["packing"].setText("")
        self.tables[index]["order_number"].setText("")

    # -------------------------
    # 재고 관련 (필요시)
    # -------------------------
    def send_warning(self, message):
        msg = String()
        msg.data = message
        self.publisher_.publish(msg)

    def check_inventory(self, item, quantity):
        if quantity <= 0:
            self.send_warning(f"{item} 재고 없음: {quantity}")
        elif quantity <= 5:
            self.send_warning(f"{item} 재고 부족: {quantity}")

    # -------------------------
    # Nav2 액션 클라이언트 로직
    # -------------------------
    def send_goal_to_nav2(self, x, y, z, w, callback_when_done=None):
        """
        (x, y, z, w)로 이동 Goal을 Nav2 액션 서버에 전송.
        callback_when_done(status)가 있으면,
        이동이 끝났을 때 GoalStatus를 인자로 호출한다.
        """
        # 액션 서버가 뜰 때까지 대기 (필요시 Timeout 설정 가능)
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("NavigateToPose action server not available!")
            if callback_when_done:
                callback_when_done(GoalStatus.STATUS_ABORTED)
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        # TODO: 필요하다면 현재 시각 넣기 (header.stamp = ...)
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.z = z
        goal_msg.pose.pose.orientation.w = w

        send_goal_future = self.nav_to_pose_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        def _goal_response_callback(future):
            goal_handle = future.result()
            if not goal_handle or not goal_handle.accepted:
                self.get_logger().warn("Goal was rejected by server")
                if callback_when_done:
                    callback_when_done(GoalStatus.STATUS_ABORTED)
                return
            self.get_logger().info("Goal accepted by server, waiting for result...")
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(partial(self._result_callback, callback_when_done))

        send_goal_future.add_done_callback(_goal_response_callback)

    def feedback_callback(self, feedback):
        # 여기서 feedback_msg.feedback에 세부 정보가 들어있음
        pass  # 필요시 self.get_logger().info(...) 로 진행 상황 로깅 가능

    def _result_callback(self, callback_when_done, future):
        result = future.result()
        if not result:
            self.get_logger().warn("Result is None")
            if callback_when_done:
                callback_when_done(GoalStatus.STATUS_UNKNOWN)
            return

        status = result.status
        if callback_when_done:
            callback_when_done(status)

# -----------------------------------
# main
# -----------------------------------
def main(args=None):
    rclpy.init(args=args)

    app = QApplication(sys.argv)
    kitchen_window = KitchenApp()
    kitchen_window.show()

    # ROS를 별도 스레드에서 spin
    ros_thread = threading.Thread(target=rclpy.spin, args=(kitchen_window,), daemon=True)
    ros_thread.start()

    # Ctrl + C 시그널 처리
    signal.signal(signal.SIGINT, signal.SIG_DFL)

    try:
        sys.exit(app.exec_())
    except KeyboardInterrupt:
        pass
    finally:
        kitchen_window.destroy_node()
        rclpy.shutdown()
        ros_thread.join()
