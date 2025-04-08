import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
import time

class FollowingSystem(Node):
    def __init__(self):
        super().__init__('following_system')
        # bounding_box/center 토픽 구독 (YOLO 노드에서 발행)
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'bounding_box/center',
            self.listener_callback,
            10
        )
        # cmd_vel 토픽으로 로봇 속도 명령 발행
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # 데이터 수신 관련 변수
        self.last_received_time = time.time()
        self.timeout_duration = 2.0

        # 카메라 해상도 (YOLO 노드 기준: 640x480)
        self.image_width = 640
        self.image_height = 480
        
        # 좌우, 상하 오차 임계값 (픽셀 단위)
        self.threshold_x = 40  
        self.threshold_y = 40  
        
        # 속도 파라미터
        self.angular_speed = 0.05       # 좌우 오차에 따른 회전 속도
        self.angular_speed_search = 0.15 # 대상이 없을 때 제자리 회전 속도
        self.linear_speed_far = 0.10    # 대상이 멀면 전진 속도
        self.linear_speed_close = 0.05  # 대상이 중간 거리일 때 전진 속도
        self.linear_speed_stop = 0.0    # 정지
        
        # 바운딩 박스 높이 기준 (대략적인 거리 추정을 위한 임계값)
        self.height_far = 140    # 높이가 이 값보다 작으면 대상이 멀다고 판단
        self.height_close = 180  # 높이가 이 값보다 크면 대상이 너무 가까워 후진 필요

    def listener_callback(self, msg):
        try:
            current_time = time.time()
            data = msg.data

            # [center_x, center_y, height] 데이터가 반드시 3개 이상 포함되어야 함
            if len(data) < 3:
                self.get_logger().warn("Received data invalid: need [center_x, center_y, height]")
                return
            
            center_x = float(data[0])
            center_y = float(data[1])
            height   = float(data[2])
            self.last_received_time = current_time

            twist = Twist()

            # 대상이 없으면 (모든 값이 0) 제자리 회전
            if center_x == 0.0 and center_y == 0.0:
                twist.linear.x = 0.0
                twist.angular.z = self.angular_speed_search
                self.get_logger().info("No target detected => rotating in place")
            else:
                # 이미지 중앙 (320,240) 기준 오차 계산
                mid_x = self.image_width / 2.0
                error_x = center_x - mid_x

                # 좌우 오차에 따라 회전 명령 결정
                if error_x < -self.threshold_x:
                    twist.angular.z = +self.angular_speed
                elif error_x > self.threshold_x:
                    twist.angular.z = -self.angular_speed
                else:
                    twist.angular.z = 0.0

                # 바운딩 박스 높이(대략적인 거리 추정)에 따른 선속도 결정
                if height < self.height_far:
                    twist.linear.x = self.linear_speed_far
                elif height > self.height_close:
                    twist.linear.x = -self.linear_speed_close
                else:
                    twist.linear.x = self.linear_speed_close

                self.get_logger().info(f"Target detected: center=({center_x:.1f},{center_y:.1f}), height={height:.1f}")
                self.get_logger().info(f"Computed error_x={error_x:.1f} => angular={twist.angular.z}, linear={twist.linear.x}")

            # cmd_vel 토픽에 Twist 메시지 발행
            self.cmd_vel_publisher.publish(twist)
        except Exception as e:
            self.get_logger().error(f"Error in listener_callback: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = FollowingSystem()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down FollowingSystem node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
