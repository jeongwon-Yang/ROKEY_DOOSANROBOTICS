import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped
import cv2
import numpy as np
import math
from queue import Queue

class TurtleBot4PathPlanner(Node):
    def __init__(self):
        super().__init__('turtlebot4_path_planner')

        # ROS2 Publishers and Subscribers
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Parameters
        self.resolution = 0.02  # Map resolution (meters per pixel)
        self.origin = (0.0, 0.0)  # Map origin (world coordinates)
        self.robot_radius = 0.2  # Robot radius (meters)
        self.robot_pixel_radius = int(self.robot_radius / self.resolution)  # Pixel radius
        self.goal_tolerance = 0.1  # Tolerance for reaching goal (meters)

        # State
        self.robot_position = None
        self.robot_yaw = 0.0
        self.map_image = None
        self.visited = None

        # Load map and preprocess
        self.load_map('/home/yangjeongwon/SlamTurtlebot_ws/src/turtlebot_exploration_pkg/map/turtlebot4_map.pgm')
        self.get_logger().info("TurtleBot4PathPlanner initialized.")

    def load_map(self, map_image_path):
        self.map_image = cv2.imread(map_image_path, cv2.IMREAD_GRAYSCALE)
        if self.map_image is None:
            self.get_logger().error("Failed to load map image.")
            return

        self.visited = np.zeros_like(self.map_image, dtype=np.uint8)  # To track visited cells
        self.get_logger().info(f"Map loaded: {map_image_path}")

    def odom_callback(self, msg):
        # Update robot position
        self.robot_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        )
        # Update robot yaw (orientation)
        q = msg.pose.pose.orientation
        self.robot_yaw = math.degrees(math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        ))

    def world_to_pixel(self, world_x, world_y):
        pixel_x = int((world_x - self.origin[0]) / self.resolution)
        pixel_y = int((world_y - self.origin[1]) / self.resolution)
        return pixel_x, pixel_y

    def pixel_to_world(self, pixel_x, pixel_y):
        world_x = pixel_x * self.resolution + self.origin[0]
        world_y = pixel_y * self.resolution + self.origin[1]
        return world_x, world_y

    def rotate_map(self, map_image, angle):
        center = (map_image.shape[1] // 2, map_image.shape[0] // 2)
        rotation_matrix = cv2.getRotationMatrix2D(center, angle, 1.0)
        rotated_map = cv2.warpAffine(map_image, rotation_matrix, (map_image.shape[1], map_image.shape[0]))
        return rotated_map
        
    

    def bfs_find_path(self):
        if self.robot_position is None:
            self.get_logger().warn("Robot position is not available.")
            return None

        start_pixel = self.world_to_pixel(*self.robot_position)
        queue = Queue()
        queue.put(start_pixel)
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]

        while not queue.empty():
            current = queue.get()
            for d in directions:
                neighbor = (current[0] + d[0], current[1] + d[1])
                if self.is_valid_cell(neighbor):
                    queue.put(neighbor)
                    self.visited[neighbor[1], neighbor[0]] = 1

        return None  # BFS is a placeholder to expand later

    def is_valid_cell(self, cell):
        x, y = cell
        if x < 0 or y < 0 or x >= self.map_image.shape[1] or y >= self.map_image.shape[0]:
            return False
        if self.map_image[y, x] == 0:  # Obstacle
            return False
        if self.visited[y, x] == 1:  # Already visited
            return False
        return True

    def update_map(self):
        if self.robot_position:
            # Rotate the map to align with the robot's yaw
            rotated_map = self.rotate_map(self.map_image, -self.robot_yaw)
            pixel_robot = self.world_to_pixel(*self.robot_position)
            cv2.circle(rotated_map, pixel_robot, self.robot_pixel_radius, (0, 255, 0), -1)  # Green path
            self.publish_map(rotated_map)

    def publish_map(self, rotated_map):
        height, width = rotated_map.shape
        map_msg = OccupancyGrid()
        map_msg.header.stamp = self.get_clock().now().to_msg()
        map_msg.header.frame_id = "map"
        map_msg.info.resolution = self.resolution
        map_msg.info.width = width
        map_msg.info.height = height
        map_msg.info.origin.position.x = self.origin[0]
        map_msg.info.origin.position.y = self.origin[1]
        map_msg.info.origin.orientation.w = 1.0

        flattened_map = rotated_map.flatten()
        map_msg.data = [int(val) for val in flattened_map]

        self.map_pub.publish(map_msg)
        self.get_logger().info("Published updated map.")

    def execute(self):
        while rclpy.ok():
            self.bfs_find_path()
            self.update_map()
            rclpy.spin_once(self)

def main(args=None):
    rclpy.init(args=args)
    node = TurtleBot4PathPlanner()
    try:
        node.execute()
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user. Shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
