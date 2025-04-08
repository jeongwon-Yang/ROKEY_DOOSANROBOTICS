#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import threading
import time
import json

class ConveyorBridge(Node):
    def __init__(self):
        super().__init__('conveyor_bridge')

        # (1) 컨베이어 상태 퍼블리셔: 아두이노에서 받은 상태를 퍼블리시합니다.
        self.status_pub = self.create_publisher(String, 'conveyor/status', 10)

        # (2) 컨베이어 제어 구독자: GUI나 오케스트라 노드에서 보내는 제어 명령을 수신합니다.
        self.control_sub = self.create_subscription(
            String,
            'conveyor/control',
            self.control_callback,
            10
        )

        # (3) 시리얼 포트 연결: 아두이노와의 통신을 위한 시리얼 포트 설정
        try:
            self.serial_port = serial.Serial('/dev/ttyACM0', 115200, timeout=0.1)
            self.get_logger().info("Serial port opened successfully.")
        except Exception as e:
            self.get_logger().error(f"Serial port open failed: {e}")
            self.serial_port = None

        # (4) 시리얼 데이터 읽기 스레드 시작: 아두이노에서 보내는 데이터를 계속 읽습니다.
        if self.serial_port is not None and self.serial_port.is_open:
            self.thread = threading.Thread(target=self.read_serial_loop)
            self.thread.daemon = True
            self.thread.start()

    def read_serial_loop(self):
        """
        아두이노로부터 데이터를 지속적으로 읽어와서 상태 토픽('conveyor/status')으로 퍼블리시합니다.
        """
        while True:
            try:
                if self.serial_port.in_waiting > 0:
                    data = self.serial_port.read(self.serial_port.in_waiting).decode('utf-8', errors='ignore')
                    data = data.strip()
                    if data:
                        self.get_logger().info(f"[Arduino] {data}")
                        # 아두이노에서 받은 데이터를 기반으로 상태 문자열을 생성
                        state = self.parse_state(data)
                        msg = String()
                        msg.data = state
                        self.status_pub.publish(msg)
                time.sleep(0.05)
            except Exception as e:
                self.get_logger().error(f"Serial read error: {e}")
                break

    def parse_state(self, data: str) -> str:
        # 데이터 내에 '_'가 있으면 "RUN", '.'가 있으면 "READY", 그 외에는 "INIT"로 판단
        if '_' in data:
            return "RUN"
        elif '.' in data:
            return "READY"
        else:
            return "INIT"

    def control_callback(self, msg: String):
        """
        /conveyor/control 토픽에서 JSON 문자열을 받아 제어 명령을 처리합니다.
        예) {"control": "go", "red_quantity": 1, "blue_quantity": 1}
            {"control": "stop"}
        """
        raw_json = msg.data.strip()
        self.get_logger().info(f"Received control: {raw_json}")
        try:
            cmd_dict = json.loads(raw_json)
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Invalid JSON: {e}")
            return

        control = cmd_dict.get("control", "")
        # 여기서 distance.mm 또는 job 등의 추가 정보도 받을 수 있으나, 현재는 단순히 "go"와 "stop"만 처리합니다.
        if control == "go":
            self.send_to_arduino("go")
        elif control == "stop":
            self.send_to_arduino("s")
        else:
            self.get_logger().warn(f"Unknown control: {control}")

    def send_to_arduino(self, command: str):
        """
        아두이노에 실제로 명령을 전송합니다.
        """
        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.write(command.encode('utf-8'))
                self.serial_port.write(b'\n')
                self.get_logger().info(f"Sent to Arduino: {command}")
            except Exception as e:
                self.get_logger().error(f"Write error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ConveyorBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutdown by user.")
    finally:
        if node.serial_port:
            node.serial_port.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

