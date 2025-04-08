import sys
import mysql.connector
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QHBoxLayout, QPushButton, QLabel, QStackedWidget, QWidget, QGridLayout, QMessageBox, QSpinBox, QComboBox, QLineEdit
from PyQt5.QtCore import Qt


class KioskGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("중화요리 키오스크")
        self.setGeometry(100, 100, 1024, 768)

        # 화면 전환을 위한 StackedWidget
        self.stacked_widget = QStackedWidget()
        self.setCentralWidget(self.stacked_widget)

        # 화면 구성
        self.create_initial_screen()
        self.create_table_selection_screen()
        self.create_menu_screen()

        # 테이블 상태 관리
        self.table_status = {f"테이블 {i + 1}": False for i in range(9)}  # False: 선택 가능, True: 선택 불가

        # 선택된 테이블
        self.selected_table = None

    def create_initial_screen(self):
        """첫 화면: 매장/포장 선택"""
        initial_screen = QWidget()
        layout = QVBoxLayout()

        # 상단 안내 텍스트
        label = QLabel("식사 장소를 선택해주세요")
        label.setAlignment(Qt.AlignCenter)
        label.setStyleSheet("font-size: 24px; font-weight: bold;")
        layout.addWidget(label)

        # 매장/포장 버튼
        button_layout = QHBoxLayout()

        dine_in_button = QPushButton("매장")
        dine_in_button.setStyleSheet("font-size: 32px; background-color: lightblue; height: 200px;")
        dine_in_button.clicked.connect(self.goto_table_selection_screen)
        button_layout.addWidget(dine_in_button)

        take_out_button = QPushButton("포장")
        take_out_button.setStyleSheet("font-size: 32px; background-color: lightgreen; height: 200px;")
        take_out_button.clicked.connect(self.goto_menu_screen)
        button_layout.addWidget(take_out_button)

        layout.addLayout(button_layout)

        # 하단 로고
        title_label = QLabel("Rokey 중국집")
        title_label.setAlignment(Qt.AlignCenter)
        title_label.setStyleSheet("font-size: 28px; color: red; font-weight: bold;")
        layout.addWidget(title_label)

        initial_screen.setLayout(layout)
        self.stacked_widget.addWidget(initial_screen)

    def create_table_selection_screen(self):
        """테이블 선택 화면"""
        table_screen = QWidget()
        layout = QVBoxLayout()

        # 안내 텍스트
        label = QLabel("테이블을 선택해주세요")
        label.setAlignment(Qt.AlignCenter)
        label.setStyleSheet("font-size: 18px;")
        layout.addWidget(label)

        # 테이블 맵
        table_grid = QGridLayout()
        self.table_buttons = {}

        # 동그라미 형태로 테이블 배치
        for i in range(9):
            table_name = f"테이블 {i + 1}"
            button = QPushButton(table_name)
            button.setStyleSheet("background-color: green; color: white; border-radius: 50px; font-size: 18px; height: 100px; width: 100px;")
            button.clicked.connect(lambda _, t=table_name: self.handle_table_selection(t))
            self.table_buttons[table_name] = button
            table_grid.addWidget(button, i // 3, i % 3)

        layout.addLayout(table_grid)
        table_screen.setLayout(layout)
        self.stacked_widget.addWidget(table_screen)

    def create_menu_screen(self):
        """메뉴 화면"""
        menu_screen = QWidget()
        layout = QVBoxLayout()

        # 메뉴 항목 예시
        self.menu_list = [
            "짜장면", "짬뽕", "잡채밥", "볶음밥", "탕수육", "군만두", "찐만두", "떡갈비", "음료수"
        ]
        self.menu_buttons = {}

        # 메뉴 선택 버튼들 생성
        for item in self.menu_list:
            button = QPushButton(item)
            button.setStyleSheet("font-size: 24px;")
            button.clicked.connect(lambda _, i=item: self.select_menu(i))
            self.menu_buttons[item] = button
            layout.addWidget(button)

        # 수량 선택기
        self.quantity_label = QLabel("수량:")
        self.quantity_spinbox = QSpinBox()
        self.quantity_spinbox.setMinimum(1)
        self.quantity_spinbox.setMaximum(10)
        layout.addWidget(self.quantity_label)
        layout.addWidget(self.quantity_spinbox)

        # 주문하기 버튼
        self.order_button = QPushButton("주문하기")
        self.order_button.setStyleSheet("font-size: 24px; background-color: lightcoral;")
        self.order_button.clicked.connect(self.submit_order)
        layout.addWidget(self.order_button)

        menu_screen.setLayout(layout)
        self.stacked_widget.addWidget(menu_screen)

    def goto_table_selection_screen(self):
        """테이블 선택 화면으로 이동"""
        self.stacked_widget.setCurrentIndex(1)

    def goto_menu_screen(self):
        """메뉴 화면으로 이동"""
        self.stacked_widget.setCurrentIndex(2)

    def handle_table_selection(self, table_name):
        """테이블 선택 처리"""
        if self.table_status[table_name]:
            # 선택 불가한 경우 알림 표시
            QMessageBox.warning(self, "테이블 선택 불가", f"{table_name}은 이미 선택되었습니다.")
        else:
            # 테이블 선택 가능: 상태 변경 및 메뉴 화면으로 이동
            self.selected_table = table_name
            self.table_status[table_name] = True
            self.table_buttons[table_name].setStyleSheet("background-color: red; color: white; border-radius: 50px;")
            QMessageBox.information(self, "테이블 선택", f"{table_name}을 선택하셨습니다.")
            self.goto_menu_screen()

    def select_menu(self, item):
        """메뉴 선택 처리"""
        self.selected_menu = item
        QMessageBox.information(self, "메뉴 선택", f"{item}을 선택하셨습니다.")

    def submit_order(self):
        """주문 제출 및 MySQL 데이터베이스에 저장"""
        quantity = self.quantity_spinbox.value()
        menu_item = self.selected_menu
        table_number = self.selected_table

        # 가격 계산 (예시)
        price = 0
        if menu_item == "짜장면":
            price = 6000 * quantity
        elif menu_item == "짬뽕":
            price = 7000 * quantity
        elif menu_item == "잡채밥":
            price = 7000 * quantity
        elif menu_item == "볶음밥":
            price = 7000 * quantity
        elif menu_item == "탕수육":
            price = 15000 * quantity
        elif menu_item == "군만두":
            price = 2000 * quantity
        elif menu_item == "찐만두":
            price = 3000 * quantity
        elif menu_item == "떡갈비":
            price = 3000 * quantity
        elif menu_item == "음료수":
            price = 2000 * quantity

        try:
            # MySQL 데이터베이스 연결
            db = mysql.connector.connect(
                host="localhost",
                user="root",
                password="mysql",  # 실제 MySQL 비밀번호 입력
                database="mydatabase"  # 사용할 데이터베이스명
            )

            cursor = db.cursor()

            # 'order_id' 생성
            cursor.execute("SELECT MAX(id) FROM orders")
            max_id = cursor.fetchone()[0]
            new_order_id = f"kicht-{max_id + 1}"

            # SQL 쿼리 작성: 테이블 번호, 메뉴, 가격 정보 삽입
            query = "INSERT INTO orders (table_number, menu_item, quantity, total_price, order_id) VALUES (%s, %s, %s, %s, %s)"
            cursor.execute(query, (table_number, menu_item, quantity, price, new_order_id))  # 쿼리 실행

            db.commit()  # 변경 사항을 데이터베이스에 커밋하여 저장

            # 주문 성공 메시지
            QMessageBox.information(self, "주문 완료", f"{menu_item} 주문이 완료되었습니다.\n총 가격: {price} 원")

            # UI에서 입력 필드 초기화
            self.quantity_spinbox.setValue(1)

        except mysql.connector.Error as err:
            print(f"Error: {err}")
            QMessageBox.critical(self, "오류", "주문을 처리하는 중 오류가 발생했습니다.")
        finally:
            # MySQL 연결 종료
            cursor.close()
            db.close()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = KioskGUI()
    window.show()
    sys.exit(app.exec_())