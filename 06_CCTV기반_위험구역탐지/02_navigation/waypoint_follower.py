import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Quaternion, PoseWithCovarianceStamped
from nav2_msgs.action import FollowWaypoints
from std_msgs.msg import String
import math

class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')
        self.object_detected = False  # 초기화: 객체 감지 여부를 False로 설정
        self.action_client = ActionClient(self, FollowWaypoints, '/follow_waypoints')
        self.initial_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.initial_pose_callback,
            10
        )
        self.object_detected_sub = self.create_subscription(
            String,
            '/object_detected',
            self.object_detected_callback,
            10
        )
        self.get_logger().info('Waiting for initial pose...')

    def initial_pose_callback(self, msg):
        self.get_logger().info(
            f'Received initial pose: x={msg.pose.pose.position.x}, '
            f'y={msg.pose.pose.position.y}, '
            f'orientation_z={msg.pose.pose.orientation.z}, '
            f'orientation_w={msg.pose.pose.orientation.w}'
        )
        self.get_logger().info('Starting waypoint navigation...')
        self.send_goal()

    def object_detected_callback(self, msg):
        if self.object_detected:
            self.get_logger().info(f'Object detected: {msg.data}')
            self.get_logger().info('Object detected, shutting down the node...')
            self.destroy_node()  # ROS2 노드 종료
            rclpy.shutdown()     # ROS2 시스템 종료

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)

    def send_goal(self):
        waypoints = []
        waypoint_data = [
            (0.20662596156342655, -0.4871472785139167, -1.17),
            (-0.5098931522807715, -0.5766481689018857, 3.017),
            (-0.7339190729114683, -0.366880572197456, 2.384),
            (-1.2838784688896316, -0.2265589907963823, 2.893),
            (-1.4204211936158202, -0.5974287556500998, -1.930),
            (-1.5243525436303194, -0.6399604662931665, -2.748)
        ]
        for x, y, yaw in waypoint_data:
            waypoint = PoseStamped()
            waypoint.header.frame_id = "map"
            waypoint.pose.position.x = x
            waypoint.pose.position.y = y
            waypoint.pose.position.z = 0.0
            waypoint.pose.orientation = self.euler_to_quaternion(0, 0, yaw)
            waypoints.append(waypoint)

        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = waypoints

        self.action_client.wait_for_server()
        self._send_goal_future = self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        self.get_logger().info('Goal accepted :)')
        self._goal_handle = goal_handle
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Current Waypoint Index: {feedback.current_waypoint}')

    def get_result_callback(self, future):
        result = future.result().result
        missed_waypoints = result.missed_waypoints
        if missed_waypoints:
            self.get_logger().info(f'Missed waypoints: {missed_waypoints}')
        else:
            self.get_logger().info('All waypoints completed successfully!')

def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollower()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
