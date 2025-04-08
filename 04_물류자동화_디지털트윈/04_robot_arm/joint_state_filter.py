import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math

class JointStateFilterNode(Node):
    def __init__(self):
        super().__init__('joint_state_filter_node')

        # 구독할 토픽과 퍼블리시할 토픽 설정
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        self.publisher = self.create_publisher(JointState, '/filtered_joint_states', 10)
        self.get_logger().info("JointStateFilterNode is running...")

    def joint_state_callback(self, msg):
        # 새로운 JointState 메시지 생성
        filtered_msg = JointState()
        filtered_msg.header = msg.header
        filtered_msg.name = msg.name
        filtered_msg.position = msg.position
        filtered_msg.velocity = msg.velocity
        
        # .nan 값을 0.0으로 변경
        filtered_msg.effort = [0.0 if math.isnan(e) else e for e in msg.effort]

        # 퍼블리시
        self.publisher.publish(filtered_msg)
        self.get_logger().info(f"Published filtered_joint_states with efforts: {filtered_msg.effort}")

def main(args=None):
    rclpy.init(args=args)
    node = JointStateFilterNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node terminated by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
