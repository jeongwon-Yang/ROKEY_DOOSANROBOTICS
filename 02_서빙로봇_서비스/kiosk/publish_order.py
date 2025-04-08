import sys
import threading
import queue
import rclpy
import signal
from rclpy.node import Node
from rclpy.qos import QoSProfile


from PySide2.QtCore import *
from PySide2.QtGui import *
from PySide2.QtWidgets import *

from std_msgs.msg import String

class NODE(Node):
    def __init__(self):
        super().__init__('publisher')
        qos_profile = QoSProfile(depth=5)
        self.message_publisher = self.create_publisher(
                    String, 'message', qos_profile)

        self.queue = queue.Queue()
        self.timer = self.create_timer(0.1, self.publish_message)

    def publish_message(self):
        while not self.queue.empty():
            message = self.queue.get()
            msg = String()
            msg.data = message
            self.message_publisher.publish(msg)
            self.get_logger().info(f'Published message: {message}')


class GUI(QMainWindow):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.setupUi()
        
    def setupUi(self):
        if not self.objectName():
            self.setObjectName(u"MainWindow")
        self.resize(361, 332)

        self.centralwidget = QWidget(self)
        self.centralwidget.setObjectName(u"centralwidget")
        self.lineEdit = QLineEdit(self.centralwidget)
        self.lineEdit.setObjectName(u"lineEdit")
        self.lineEdit.setGeometry(QRect(30, 20, 131, 31))
        self.pushButton = QPushButton(self.centralwidget)
        self.pushButton.setObjectName(u"pushButton")
        self.pushButton.setGeometry(QRect(200, 20, 81, 31))
        self.pushButton.clicked.connect(self.button_clicked)

        self.setCentralWidget(self.centralwidget)

    def button_clicked(self):
        self.message = self.lineEdit.text()
        self.node.queue.put(self.message)
        self.lineEdit.clear()

def main():
    rclpy.init()
    node = NODE()
    ros_thread = threading.Thread(target=lambda : rclpy.spin(node))
    ros_thread.start()

    app = QApplication(sys.argv)
    gui = GUI(node)
    gui.show()

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