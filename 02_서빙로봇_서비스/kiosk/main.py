#!/usr/bin/env python3
# main.py
import sys
import signal
import threading
import json
import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile

from functools import partial

from PySide2.QtCore import Qt, QTimer
from PySide2.QtGui import QPixmap, QFont
from PySide2.QtWidgets import (
    QApplication, QMainWindow, QWidget, QStackedWidget, 
    QVBoxLayout, QHBoxLayout, QGridLayout, QPushButton, QLabel, 
    QMessageBox, QTableWidget, QTableWidgetItem, QTabWidget, 
    QHeaderView, QLineEdit
)

# =================================
# 1. ROS Publisher Node (NODE)
# =================================
class NODE(Node):
    def __init__(self):
        super().__init__('order_publisher')
        qos_profile = QoSProfile(depth=5)
        self.publisher_ = self.create_publisher(String, 'order_topic', qos_profile)
        self.get_logger().info('Publisher Node started.')

    def publish_order(self, order_data: dict):
        """
        order_data 예시:
        {
            "table_number": "테이블 1",
            "menu_items": [{"name": "유니짜장", "quantity": 2}, ...],
            "is_takeout": False
        }
        """
        msg = String()
        msg.data = json.dumps(order_data, ensure_ascii=False)
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published order: {msg.data}")


# =================================
# 2. 메뉴/장바구니/결제 화면 (QWidget)
# =================================
class MenuScreen(QWidget):
    def __init__(self, parent=None):
        """
        parent에는 KioskGUI(QMainWindow)가 들어온다고 가정.
        """
        super().__init__(parent)
        self.kiosk_gui = parent            # 메인 QMainWindow 객체 참조
        self.publisher_node = None         # ROS 퍼블리셔 노드 (외부 주입)
        self.selected_table = "테이블 1"
        self.is_takeout = False

        self.total_price = 0
        self.current_tab_data = {}

        self.init_ui()

    def init_ui(self):
        main_layout = QHBoxLayout(self)
        self.setLayout(main_layout)

        # 좌측 탭 메뉴
        self.tab_widget = QTabWidget()
        self.tab_widget.currentChanged.connect(self.tab_changed)
        self.tab_widget.setStyleSheet("QTabWidget::pane { border: 1px solid #dcdcdc; }")
        main_layout.addWidget(self.tab_widget)

        # 우측 레이아웃
        right_layout = QVBoxLayout()
        main_layout.addLayout(right_layout)

        # 장바구니 테이블
        self.cart_table = QTableWidget(0, 4)
        self.cart_table.setHorizontalHeaderLabels(["메뉴", "수량", "가격", ""])
        header = self.cart_table.horizontalHeader()
        header.setSectionResizeMode(0, QHeaderView.Stretch)
        header.setSectionResizeMode(1, QHeaderView.ResizeToContents)
        header.setSectionResizeMode(2, QHeaderView.ResizeToContents)
        header.setSectionResizeMode(3, QHeaderView.ResizeToContents)
        self.cart_table.setStyleSheet("QTableWidget { background-color: #f8f8f8; font-size: 14px; }")
        right_layout.addWidget(self.cart_table)

        # 총 가격 및 결제 버튼
        self.total_price_label = QLabel("총 가격: 0원")
        self.total_price_label.setAlignment(Qt.AlignRight)
        self.total_price_label.setFont(QFont('Arial', 14, QFont.Bold))
        right_layout.addWidget(self.total_price_label)

        self.checkout_button = QPushButton("결제하기")
        self.checkout_button.setStyleSheet("background-color: #4CAF50; color: white; font-size: 18px; padding: 10px;")
        self.checkout_button.clicked.connect(self.checkout_order)
        right_layout.addWidget(self.checkout_button)

        # 메뉴 탭 추가 (이미지 경로를 get_image_path로 수정)
        self.add_menu_tab("식사류", [
            ("유니짜장", 7000, self.get_image_path("유니짜장.jpeg")),
            ("볶음밥", 8000, self.get_image_path("볶음밥.jpeg")),
            ("차돌짬뽕", 9000, self.get_image_path("차돌짬뽕.jpeg")),
            ("잡채밥", 8000, self.get_image_path("잡채밥.jpeg")),
            ("불짬뽕", 10000, self.get_image_path("불짬뽕.jpeg")),
            ("불짜장", 9500, self.get_image_path("불짜장.jpeg")),
        ], items_per_page=4)

        self.add_menu_tab("사이드 메뉴", [
            ("군만두", 2000, self.get_image_path("군만두.jpeg")),
            ("찐만두", 3000, self.get_image_path("찐만두.jpeg")),
            ("탕수육", 15000, self.get_image_path("탕수육.jpeg")),
        ], items_per_page=4)

        self.add_menu_tab("음료", [
            ("콜라", 2000, self.get_image_path("콜라.jpeg")),
            ("제로콜라", 2000, self.get_image_path("제로콜라.jpeg")),
            ("사이다", 2000, self.get_image_path("사이다.jpeg")),
        ], items_per_page=4)

    def get_image_path(self, filename):
        current_dir = os.path.dirname(os.path.abspath(__file__))
        image_dir = os.path.abspath(os.path.join(current_dir, "..", "picture"))
        path = os.path.join(image_dir, filename)
        if not os.path.isfile(path):
            print(f"경고: 이미지 파일이 존재하지 않습니다: {path}")
        return path

    def set_publisher_node(self, node: NODE):
        self.publisher_node = node

    def set_table_and_takeout(self, table_name: str, is_takeout: bool):
        self.selected_table = table_name
        self.is_takeout = is_takeout

    # ----------------
    # 메뉴 탭/페이징
    # ----------------
    def add_menu_tab(self, category_name, menu_items, items_per_page):
        tab_data = {
            "menu_items": menu_items,
            "items_per_page": items_per_page,
            "current_page": 0,
        }
        self.current_tab_data[category_name] = tab_data

        tab = QWidget()
        tab_layout = QVBoxLayout(tab)
        menu_grid = QGridLayout()
        tab_layout.addLayout(menu_grid)
        tab_data["menu_grid"] = menu_grid

        # 페이지 이동 버튼
        nav_layout = QHBoxLayout()
        prev_btn = QPushButton("이전")
        next_btn = QPushButton("다음")
        prev_btn.setStyleSheet("background-color: #ccc; font-size: 16px; padding: 10px;")
        next_btn.setStyleSheet("background-color: #ccc; font-size: 16px; padding: 10px;")
        prev_btn.clicked.connect(lambda: self.prev_page(category_name))
        next_btn.clicked.connect(lambda: self.next_page(category_name))
        nav_layout.addWidget(prev_btn)
        nav_layout.addWidget(next_btn)
        tab_layout.addLayout(nav_layout)

        self.tab_widget.addTab(tab, category_name)
        self.display_page(category_name, 0)

    def display_page(self, category_name, page):
        tab_data = self.current_tab_data[category_name]
        menu_items = tab_data["menu_items"]
        items_per_page = tab_data["items_per_page"]
        menu_grid = tab_data["menu_grid"]

        # 그리드 초기화
        for i in reversed(range(menu_grid.count())):
            w = menu_grid.itemAt(i).widget()
            if w:
                w.deleteLater()

        start_index = page * items_per_page
        end_index = min(start_index + items_per_page, len(menu_items))

        for i, (name, price, image_path) in enumerate(menu_items[start_index:end_index]):
            item_widget = QWidget()
            item_layout = QVBoxLayout(item_widget)

            image_label = QLabel()
            pixmap = QPixmap(image_path)
            image_label.setPixmap(pixmap.scaled(150, 150, Qt.KeepAspectRatio))
            image_label.setAlignment(Qt.AlignCenter)

            # 이미지 클릭 -> add_to_cart
            image_label.mousePressEvent = lambda ev, n=name, p=price: self.add_to_cart(n, p)

            name_label = QLabel(f"{name}\n{price}원")
            name_label.setAlignment(Qt.AlignCenter)
            name_label.setFont(QFont('Arial', 12))

            item_layout.addWidget(image_label)
            item_layout.addWidget(name_label)

            menu_grid.addWidget(item_widget, i // 2, i % 2)

        tab_data["current_page"] = page

    def prev_page(self, category_name):
        tab_data = self.current_tab_data[category_name]
        if tab_data["current_page"] > 0:
            self.display_page(category_name, tab_data["current_page"] - 1)

    def next_page(self, category_name):
        tab_data = self.current_tab_data[category_name]
        max_page = (len(tab_data["menu_items"]) - 1) // tab_data["items_per_page"]
        if tab_data["current_page"] < max_page:
            self.display_page(category_name, tab_data["current_page"] + 1)

    def tab_changed(self, index):
        category_name = self.tab_widget.tabText(index)
        current_page = self.current_tab_data[category_name]["current_page"]
        self.display_page(category_name, current_page)

    # ----------------
    # 장바구니 & 결제
    # ----------------
    def add_to_cart(self, name, price):
        # 이미 있는지 확인
        for row in range(self.cart_table.rowCount()):
            if self.cart_table.item(row, 0).text() == name:
                # 수량+1
                q_item = self.cart_table.item(row, 1)
                q = int(q_item.text()) + 1
                q_item.setText(str(q))

                # 가격 업데이트
                p_item = self.cart_table.item(row, 2)
                total_p = q * price
                p_item.setText(f"{total_p}원")

                self.total_price += price
                self.total_price_label.setText(f"총 가격: {self.total_price}원")
                return

        # 새 행 추가
        row_idx = self.cart_table.rowCount()
        self.cart_table.insertRow(row_idx)
        self.cart_table.setItem(row_idx, 0, QTableWidgetItem(name))
        self.cart_table.setItem(row_idx, 1, QTableWidgetItem("1"))
        self.cart_table.setItem(row_idx, 2, QTableWidgetItem(f"{price}원"))

        # +/- 버튼
        dec_btn = QPushButton("-")
        inc_btn = QPushButton("+")

        dec_btn.setStyleSheet("background-color: #ccc; font-size: 18px; padding: 5px;")
        inc_btn.setStyleSheet("background-color: #ccc; font-size: 18px; padding: 5px;")

        dec_btn.clicked.connect(lambda r=row_idx, p=price: self.update_quantity(r, p, -1))
        inc_btn.clicked.connect(lambda r=row_idx, p=price: self.update_quantity(r, p, 1))

        ctrl_w = QWidget()
        ctrl_lay = QHBoxLayout(ctrl_w)
        ctrl_lay.setContentsMargins(0, 0, 0, 0)
        ctrl_lay.addWidget(dec_btn)
        ctrl_lay.addWidget(inc_btn)

        self.cart_table.setCellWidget(row_idx, 3, ctrl_w)

        self.total_price += price
        self.total_price_label.setText(f"총 가격: {self.total_price}원")

    def update_quantity(self, row, price, change):
        if row >= self.cart_table.rowCount():
            return
        q_item = self.cart_table.item(row, 1)
        if not q_item:
            return
        quantity = int(q_item.text()) + change

        if quantity <= 0:
            # 제거
            old_q = int(q_item.text())
            self.total_price -= (old_q * price)
            self.cart_table.removeRow(row)
        else:
            # 갱신
            q_item.setText(str(quantity))
            p_item = self.cart_table.item(row, 2)
            new_price = quantity * price
            p_item.setText(f"{new_price}원")

            self.total_price += (price * change)
        self.total_price_label.setText(f"총 가격: {self.total_price}원")

    def checkout_order(self):
        if self.cart_table.rowCount() == 0:
            QMessageBox.warning(self, "경고", "장바구니가 비어 있습니다.")
            return

        order_summary = "\n".join([
            f"{self.cart_table.item(r, 0).text()} - {self.cart_table.item(r, 1).text()}개"
            for r in range(self.cart_table.rowCount())
        ])
        total = self.total_price

        msg = QMessageBox(self)
        msg.setWindowTitle("결제 방식")
        msg.setText(
            f"주문 내역:\n{order_summary}\n\n"
            f"총 가격: {total}원\n\n결제 방식을 선택하세요."
        )

        # 카드 결제 버튼
        card_button = QPushButton("카드")
        card_button.setStyleSheet("background-color: #4CAF50; font-size: 18px; color: white; padding: 10px;")
        card_button.clicked.connect(self.card_payment_process)
        msg.addButton(card_button, QMessageBox.AcceptRole)

        # 현금 결제 버튼
        cash_button = QPushButton("현금")
        cash_button.setStyleSheet("background-color: #ff6347; font-size: 18px; color: white; padding: 10px;")
        cash_button.clicked.connect(self.cash_payment_process)
        msg.addButton(cash_button, QMessageBox.RejectRole)

        msg.exec_()

    def card_payment_process(self):
        complete_msg = QMessageBox(self)
        complete_msg.setWindowTitle("결제 안내")
        complete_msg.setText("카드를 삽입하고 결제를 완료하세요.")
        complete_button = QPushButton("결제 완료")
        complete_button.setStyleSheet("background-color: #4CAF50; font-size: 18px; color: white; padding: 10px;")
        complete_button.clicked.connect(self.complete_payment)
        complete_msg.addButton(complete_button, QMessageBox.AcceptRole)
        complete_msg.exec_()

    def cash_payment_process(self):
        QMessageBox.information(self, "결제 안내", "카운터로 가시면 결제 도와드리겠습니다.")
        self.complete_payment()

    def complete_payment(self):
        QMessageBox.information(self, "결제 완료", "결제가 완료되었습니다.")
        # 주문 발행
        self.send_order_to_kitchen()

        # 장바구니 초기화
        self.cart_table.setRowCount(0)
        self.total_price = 0
        self.total_price_label.setText("총 가격: 0원")

        # 포장이 아닌 경우(매장 테이블)에 한해 테이블 상태 변경
        if not self.is_takeout and self.kiosk_gui:
            # 이미 'handle_table_selection'에서 table_status가 True로 설정되긴 했지만
            # 혹시 모를 예외 상황을 고려해 다시 한 번 확실히 설정
            self.kiosk_gui.table_status[self.selected_table] = True

        # 결제 완료 후, 메인화면(KioskGUI)의 초기 화면으로 돌아가기
        if self.kiosk_gui:
            self.kiosk_gui.goto_initial_screen()


    def send_order_to_kitchen(self):
        if not self.publisher_node:
            QMessageBox.warning(self, "ROS 오류", "퍼블리셔 노드가 없습니다.")
            return

        menu_items = []
        for row in range(self.cart_table.rowCount()):
            name = self.cart_table.item(row, 0).text()
            quantity = int(self.cart_table.item(row, 1).text())
            menu_items.append({"name": name, "quantity": quantity})

        order_data = {
            "table_number": self.selected_table,
            "menu_items": menu_items,
            "is_takeout": self.is_takeout
        }

        self.publisher_node.publish_order(order_data)


# =================================
# 3. 메인 QMainWindow (첫 화면 + 테이블 선택 + MenuScreen)
# =================================
class KioskGUI(QMainWindow):
    def __init__(self, node):
        super().__init__()
        self.publisher_node = node
        self.selected_table = None
        self.is_takeout = False

        self.setWindowTitle("중화요리 키오스크 - Publisher 통합버전")
        self.setGeometry(100, 100, 1024, 768)

        # 테이블 상태 (False: 아직 주문 전, True: 주문 완료/사용중)
        self.table_status = {f"테이블 {i + 1}": False for i in range(9)}

        # StackedWidget
        self.stacked_widget = QStackedWidget()
        self.setCentralWidget(self.stacked_widget)

        # 1) 첫 화면
        self.initial_screen = self.create_initial_screen()
        self.stacked_widget.addWidget(self.initial_screen)

        # 2) 테이블 선택
        self.table_screen = self.create_table_selection_screen()
        self.stacked_widget.addWidget(self.table_screen)

        # 3) 메뉴 화면 (MenuScreen)
        self.menu_screen = MenuScreen(self)  # self(즉, KioskGUI)를 MenuScreen에 전달
        self.menu_screen.set_publisher_node(self.publisher_node)
        self.stacked_widget.addWidget(self.menu_screen)

        self.stacked_widget.setCurrentIndex(0)

    # -----------------------------
    # 첫 화면: 매장/포장 선택
    # -----------------------------
    def create_initial_screen(self):
        w = QWidget()
        layout = QVBoxLayout(w)

        label = QLabel("식사 장소를 선택해주세요")
        label.setAlignment(Qt.AlignCenter)
        label.setStyleSheet("font-size: 24px; font-weight: bold; color: #333;")
        layout.addWidget(label)

        btn_layout = QHBoxLayout()
        dine_in_btn = QPushButton("매장")
        dine_in_btn.setStyleSheet("font-size: 32px; background-color: #4CAF50; color: white; height: 200px;")
        dine_in_btn.clicked.connect(self.goto_table_selection_screen)
        btn_layout.addWidget(dine_in_btn)

        take_out_btn = QPushButton("포장")
        take_out_btn.setStyleSheet("font-size: 32px; background-color: #FF6347; color: white; height: 200px;")
        take_out_btn.clicked.connect(self.goto_menu_direct_takeout)
        btn_layout.addWidget(take_out_btn)

        layout.addLayout(btn_layout)

        title_label = QLabel("Rokey 중국집")
        title_label.setAlignment(Qt.AlignCenter)
        title_label.setStyleSheet("font-size: 28px; color: red; font-weight: bold;")
        layout.addWidget(title_label)

        return w

    # -----------------------------
    # 테이블 선택 화면
    # -----------------------------
    def create_table_selection_screen(self):
        w = QWidget()
        layout = QVBoxLayout(w)

        label = QLabel("테이블을 선택해주세요")
        label.setAlignment(Qt.AlignCenter)
        label.setStyleSheet("font-size: 54px; color: #333; font-weight: bold;")
        layout.addWidget(label)

        grid = QGridLayout()
        self.table_buttons = {}
        for i in range(9):
            tname = f"테이블 {i + 1}"
            btn = QPushButton(tname)
            btn.setStyleSheet(
                "background-color: #4CAF50; color: white; border-radius: 50px; "
                "font-size: 18px; height: 100px; width: 100px;"
            )

            # 이미 주문 완료된 상태라면 빨간색 + 비활성
            if self.table_status[tname]:
                btn.setStyleSheet(
                    "background-color: #FF6347; color: white; border-radius: 50px; "
                    "font-size: 18px; height: 100px; width: 100px;"
                )
                btn.setEnabled(False)

            btn.clicked.connect(partial(self.handle_table_selection, tname))
            self.table_buttons[tname] = btn
            grid.addWidget(btn, i // 3, i % 3)

        layout.addLayout(grid)
        return w

    def handle_table_selection(self, table_name):
        # 이미 True면(주문 완료) 선택 불가
        if self.table_status[table_name]:
            QMessageBox.warning(self, "테이블 선택 불가", f"{table_name}은 이미 주문되었습니다.")
            return

        # 상태 업데이트
        self.table_status[table_name] = True
        print(f"테이블 {table_name} 선택됨 - 상태: {self.table_status[table_name]}")

        self.selected_table = table_name
        # 버튼 빨간색 + 비활성
        self.table_buttons[table_name].setStyleSheet(
            "background-color: #FF6347; color: white; border-radius: 50px; "
            "font-size: 18px; height: 100px; width: 100px;"
        )
        self.table_buttons[table_name].setEnabled(False)

        QMessageBox.information(self, "테이블 선택", f"{table_name}을 선택하셨습니다.")
        self.is_takeout = False

        # 메뉴 화면에 테이블/포장 정보 전달
        self.menu_screen.set_table_and_takeout(table_name, self.is_takeout)
        self.goto_menu_screen()

    # -----------------------------
    # 화면 전환 함수
    # -----------------------------
    def goto_table_selection_screen(self):
        self.stacked_widget.setCurrentIndex(1)

    def goto_menu_screen(self):
        self.stacked_widget.setCurrentIndex(2)

    def goto_menu_direct_takeout(self):
        self.is_takeout = True
        self.selected_table = "포장 손님"
        self.menu_screen.set_table_and_takeout(self.selected_table, self.is_takeout)
        self.goto_menu_screen()

    def goto_initial_screen(self):
        """
        결제 완료 후 혹은 다른 상황에서 '첫 화면'으로 돌아가도록 하는 함수.
        이미 주문 완료된 테이블은 빨간색으로 표시하고 비활성화 상태를 유지한다.
        주문 전인 테이블은 다시 활성화(초록색).
        """
        self.stacked_widget.setCurrentIndex(0)

        # 테이블 색상/상태 갱신
        for table_name, status in self.table_status.items():
            btn = self.table_buttons[table_name]
            if status:
                # 이미 선택/주문 완료된 테이블
                btn.setStyleSheet(
                    "background-color: #FF6347; color: white; border-radius: 50px; "
                    "font-size: 18px; height: 100px; width: 100px;"
                )
                btn.setEnabled(False)
            else:
                # 아직 주문 전인 테이블
                btn.setStyleSheet(
                    "background-color: #4CAF50; color: white; border-radius: 50px; "
                    "font-size: 18px; height: 100px; width: 100px;"
                )
                btn.setEnabled(True)


# =================================
# 4. main() 함수
# =================================
def main():
    rclpy.init()
    node = NODE()

    # ROS Spin in separate thread
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    # Qt
    app = QApplication(sys.argv)
    gui = KioskGUI(node)
    gui.show()

    # Ctrl+C 시그널 처리
    signal.signal(signal.SIGINT, signal.SIG_DFL)

    try:
        sys.exit(app.exec_())
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
