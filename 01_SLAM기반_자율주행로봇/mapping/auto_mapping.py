# 찐*5 버전에 벽과의 거리기반 필터링을 강화하고 벽 근처뿐만이 아닌 맵의 경계에 가까운 프론티어도 선택하지 않게 바꾼 버전.

import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
import math

class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('frontier_explorer')
        self.map_subscriber = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )
        self.goal_publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.map_data = None
        self.map_info = None
        self.safe_distance = 2  # 벽 근처 최소 거리 (미터)
        self.wall_distance = 2  # 벽과의 거리 기준 (미터)

    def map_callback(self, msg):
        self.map_data = np.array(msg.data, dtype=np.int8).reshape((msg.info.height, msg.info.width))
        self.map_info = msg.info
        self.safe_distance = 2 / self.map_info.resolution  # 2m -> 셀 단위로 변환
        self.wall_distance = 2 / self.map_info.resolution
        self.explore_map()

    def explore_map(self):
        if self.map_data is None:
            return

        frontiers = self.find_frontiers()
        self.get_logger().info(f"Found {len(frontiers)} frontiers before filtering.")

        frontiers = self.filter_frontiers(frontiers)
        self.get_logger().info(f"Found {len(frontiers)} frontiers after filtering.")

        if not frontiers:
            self.get_logger().info("Exploration complete. No more valid frontiers.")
            return

        goal = self.select_goal(frontiers)
        self.move_to_goal(goal)

    def find_frontiers(self):
        frontiers = []
        for y in range(1, self.map_data.shape[0] - 1):
            for x in range(1, self.map_data.shape[1] - 1):
                if self.map_data[y, x] == -1:  # 미탐사지역
                    if self.is_frontier(x, y):
                        frontiers.append((x, y))
        return frontiers

    def is_frontier(self, x, y):
        for dx in range(-1, 2):
            for dy in range(-1, 2):
                nx, ny = x + dx, y + dy
                if 0 <= nx < self.map_data.shape[1] and 0 <= ny < self.map_data.shape[0]:
                    if self.map_data[ny, nx] == 0:  # 탐사된 빈 공간
                        return True
        return False

    def filter_frontiers(self, frontiers):
        valid_frontiers = []
        for x, y in frontiers:
            if self.is_safe_frontier(x, y):
                valid_frontiers.append((x, y))
        return valid_frontiers

    def is_safe_frontier(self, x, y):
        if x < self.safe_distance or x >= self.map_data.shape[1] - self.safe_distance:
            return False
        if y < self.safe_distance or y >= self.map_data.shape[0] - self.safe_distance:
            return False

        for dx in range(-int(self.wall_distance), int(self.wall_distance) + 1):
            for dy in range(-int(self.wall_distance), int(self.wall_distance) + 1):
                nx, ny = x + dx, y + dy
                if 0 <= nx < self.map_data.shape[1] and 0 <= ny < self.map_data.shape[0]:
                    if self.map_data[ny, nx] == 100:  # 벽
                        distance = math.sqrt((dx ** 2) + (dy ** 2))
                        if distance < self.safe_distance:
                            return False
        return True

    def select_goal(self, frontiers):
        robot_x, robot_y = self.get_robot_position()

        def frontier_wall_distance(frontier):
            fx, fy = frontier
            min_distance = float('inf')
            for x in range(self.map_data.shape[1]):
                for y in range(self.map_data.shape[0]):
                    if self.map_data[y, x] == 100:  # 벽
                        distance = math.sqrt((fx - x) ** 2 + (fy - y) ** 2)
                        min_distance = min(min_distance, distance)
            return min_distance

        farthest_frontier = max(
            frontiers,
            key=lambda f: (frontier_wall_distance(f), -self.euclidean_distance(f, robot_x, robot_y))
        )

        return farthest_frontier

    def euclidean_distance(self, f, robot_x, robot_y):
        fx, fy = f
        return math.sqrt((fx - robot_x) ** 2 + (fy - robot_y) ** 2)

    def get_robot_position(self):
        robot_pose = (self.map_info.width // 2, self.map_info.height // 2)
        return robot_pose

    def move_to_goal(self, goal):
        goal_x, goal_y = goal
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = goal_x * self.map_info.resolution + self.map_info.origin.position.x
        goal_pose.pose.position.y = goal_y * self.map_info.resolution + self.map_info.origin.position.y
        goal_pose.pose.orientation.w = 1.0  # 방향은 임시로 정방향
        self.goal_publisher.publish(goal_pose)
        self.get_logger().info(f"Published goal at ({goal_x}, {goal_y}).")

def main(args=None):
    rclpy.init(args=args)
    explorer = FrontierExplorer()
    rclpy.spin(explorer)
    explorer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
