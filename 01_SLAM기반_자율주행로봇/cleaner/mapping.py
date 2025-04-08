#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan  # 혹시 필요하다면
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException

import numpy as np
from sklearn.cluster import DBSCAN

import math
import time
import os
import cv2


class TurtlebotExplorerNode(Node):
    def __init__(self):
        super().__init__('turtlebot_explorer_node')

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

        self.visited_map_pub = self.create_publisher(
            OccupancyGrid,
            '/visited_map',
            10
        )

        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.map_data = None
        self.map_width = None
        self.map_height = None
        self.map_resolution = None
        self.map_origin = (0.0, 0.0)
        self.visited_map = None

        self.is_exploring = False
        self.exploration_done = False

        self.robot_mark_radius = 0.3

        self.get_logger().info("TurtlebotExplorerNode started.")

    #-----------------------------
    # (A) 맵 콜백
    #-----------------------------
    def map_callback(self, msg: OccupancyGrid):
        w = msg.info.width
        h = msg.info.height

        if w == 0 or h == 0:
            self.get_logger().warn("Received an empty map (width=0 or height=0). Skipping initialization.")
            return

        # 맵 초기화 또는 업데이트
        if (self.map_width != w) or (self.map_height != h):
            self.map_width = w
            self.map_height = h
            self.map_resolution = msg.info.resolution
            self.map_origin = (msg.info.origin.position.x, msg.info.origin.position.y)

            map_array = np.array(msg.data, dtype=np.int8).reshape(h, w)
            self.map_data = map_array

            self.visited_map = OccupancyGrid()
            self.visited_map.header.frame_id = 'map'
            self.visited_map.info = msg.info
            self.visited_map.data = [-1] * (w * h)

            self.get_logger().info(f"Visited map initialized/re-initialized: width={w}, height={h}")
        else:
            map_array = np.array(msg.data, dtype=np.int8).reshape(h, w)
            self.map_data = map_array

        # 탐사가 완료되었으면 return
        if self.exploration_done:
            return

        # 맵 데이터 확인
        if self.map_data is None:
            return

        # 방문 맵 업데이트
        self.update_visited_map()

        # Frontier 탐색
        frontiers = self.find_frontiers_8dir(self.map_data)

        # Frontier가 없는 경우 탐사 종료
        if len(frontiers) == 0:
            self.exploration_done = True
            self.get_logger().info("No more frontiers. Exploration finished.")
            
            # 맵 저장
            self.save_map(msg)

            # 노드 종료
            self.get_logger().info("Shutting down node.")
            rclpy.shutdown()
            return

        # 탐사가 진행 중이 아니면 새로운 목표 설정
        if not self.is_exploring:
            cluster_centers, cluster_sizes = self.cluster_frontiers(frontiers)

            if not cluster_centers:
                self.exploration_done = True
                self.get_logger().info("Frontier found but no valid clusters. Stopping.")
                
                # 맵 저장
                self.save_map(msg)
                
                # 노드 종료
                self.get_logger().info("Shutting down node.")
                rclpy.shutdown()
                return

            # 가장 큰 클러스터 선택
            max_size_idx = np.argmax(cluster_sizes)
            goal_world = cluster_centers[max_size_idx]

            # 목표 전송
            self.send_goal(goal_world)
            self.is_exploring = True

        # 방문 맵 퍼블리시
        if self.visited_map is not None:
            self.visited_map.header.stamp = self.get_clock().now().to_msg()
            self.visited_map_pub.publish(self.visited_map)



    #-----------------------------
    # (C) Frontier 찾기 (8방향)
    #-----------------------------
    def is_within_bounds(self, x, y):
        return 0 <= x < self.map_width and 0 <= y < self.map_height

    def find_frontiers_8dir(self, map_array):
        frontiers = []
        h, w = map_array.shape

        for y in range(h):
            for x in range(w):
                if map_array[y, x] == 0:  # Free space
                    neighbors = self.get_neighbors_8dir(x, y, w, h)
                    for nx, ny in neighbors:
                        if map_array[ny, nx] == -1 and self.is_within_bounds(nx, ny):  # Unknown + 경계 확인
                            frontiers.append((x, y))
                            break

        # 중복 제거
        frontiers = list(set(frontiers))
        return frontiers




    def get_neighbors_8dir(self, x, y, width, height):
        neighbors = []
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue
                nx = x + dx
                ny = y + dy
                if (0 <= nx < width) and (0 <= ny < height):
                    neighbors.append((nx, ny))
        return neighbors

    #-----------------------------
    # (D) DBSCAN 클러스터링
    #-----------------------------
    def cluster_frontiers(self, frontier_points):
        if not frontier_points:
            return [], []

        X = np.array(frontier_points)
        # DBSCAN 파라미터 조정
        dbscan = DBSCAN(eps=5.0, min_samples=3).fit(X)  # eps와 min_samples는 맵 환경에 맞게 조정
        labels = dbscan.labels_

        unique_labels = set(labels)
        cluster_centers = []
        cluster_sizes = []

        for label in unique_labels:
            if label == -1:  # 노이즈 제거
                continue

            indices = np.where(labels == label)
            cluster_points = X[indices]

            # 중심 계산
            cx = np.mean(cluster_points[:, 0])
            cy = np.mean(cluster_points[:, 1])

            # 맵 좌표 → 월드 좌표 변환
            wx, wy = self.map_to_world(cx, cy)
            cluster_centers.append((wx, wy))
            cluster_sizes.append(len(cluster_points))  # 클러스터 크기 추가

        return cluster_centers, cluster_sizes


    def map_to_world(self, mx, my):
        """맵 셀 좌표(mx,my)를 map 프레임의 월드 좌표(wx,wy)로 변환"""
        wx = self.map_origin[0] + (mx + 0.5) * self.map_resolution
        # OccupancyGrid는 (0,0)이 왼쪽 아래가 아닐 수 있어 주의.
        # 그러나 여기서는 단순 계산:
        wy = self.map_origin[1] + (my + 0.5) * self.map_resolution
        return (wx, wy)

    #-----------------------------
    # (E) Navigation Goal 전송
    #-----------------------------
    def send_goal(self, target):
        goal_msg = NavigateToPose.Goal()
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()

        goal_pose.pose.position.x = float(target[0])
        goal_pose.pose.position.y = float(target[1])
        goal_pose.pose.orientation.w = 1.0  # 회전은 일단 0도로 세팅

        goal_msg.pose = goal_pose

        # 액션 서버 연결 대기
        if not self.nav_client.server_is_ready():
            self.nav_client.wait_for_server()

        # goal 전송
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        # Goal이 정상 접수되었는지 확인
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected by server.")
            self.is_exploring = False
            return

        # 탐색 진행 → 결과 콜백 등록
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.navigation_result_callback)

    def navigation_result_callback(self, future):
        result = future.result().result
        status = future.result().status

        if status == 4:  # SUCCEEDED
            # 성공
            self.get_logger().info("Goal reached.")
        else:
            self.get_logger().error(f"Goal failed with status: {status}")
            # 여기서 재시도 로직을 넣거나, 그냥 다음 사이클로 넘어감

        # 다음 Frontier 탐색을 위해 exploring 상태 해제
        self.is_exploring = False
    
    #-----------------------------
    # 맵 저장 함수
    #-----------------------------
    def save_map(self, map_data, folder_path="/home/yangjeongwon/aaajong_ws/src/turtlebot_exploration_pkg/map"):
        """
        OccupancyGrid 데이터를 OpenCV 이미지로 저장
        """
        if map_data is None:
            self.get_logger().error("No map data available to save.")
            return

        try:
            # 폴더 존재 확인 및 생성
            if not os.path.exists(folder_path):
                os.makedirs(folder_path)

            # OccupancyGrid 데이터를 numpy 배열로 변환
            map_array = np.array(map_data.data, dtype=np.int8).reshape(map_data.info.height, map_data.info.width)

            # OccupancyGrid 값 변환: [-1(Unknown), 0(Free), 100(Occupied)] → [128, 255, 0]
            map_image = np.zeros_like(map_array, dtype=np.uint8)
            map_image[map_array == -1] = 128  # Unknown
            map_image[map_array == 0] = 255  # Free
            map_image[map_array == 100] = 0  # Occupied

            # 파일명 생성
            timestamp = int(time.time())
            file_name = f"map_{timestamp}.png"
            file_path = os.path.join(folder_path, file_name)

            # 이미지 저장
            cv2.imwrite(file_path, map_image)
            self.get_logger().info(f"Map saved to: {file_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to save map: {e}")


    #-----------------------------
    # (F) visited_map 업데이트
    #-----------------------------
    def update_visited_map(self):
        if self.visited_map is None:
            self.get_logger().warn("Visited map is not initialized.")
            return

        try:
            now = rclpy.time.Time()
            transform = self.tf_buffer.lookup_transform('map', 'base_link', now)

            rx = transform.transform.translation.x
            ry = transform.transform.translation.y

            cell_x = int((rx - self.map_origin[0]) / self.map_resolution)
            cell_y = int((ry - self.map_origin[1]) / self.map_resolution)

            self.get_logger().info(f"Robot position in map cells: x={cell_x}, y={cell_y}")

            radius_in_cells = int(self.robot_mark_radius / self.map_resolution)

            for dy in range(-radius_in_cells, radius_in_cells + 1):
                for dx in range(-radius_in_cells, radius_in_cells + 1):
                    nx = cell_x + dx
                    ny = cell_y + dy

                    if (0 <= nx < self.map_width) and (0 <= ny < self.map_height):
                        dist = math.sqrt(dx * dx + dy * dy) * self.map_resolution
                        if dist <= self.robot_mark_radius:
                            index = ny * self.map_width + nx
                            if index < len(self.visited_map.data):
                                self.visited_map.data[index] = 100
                            else:
                                self.get_logger().error(f"Attempted to access index {index}, but data length is {len(self.visited_map.data)}")
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().error(f"TF lookup failed: {e}")
    
    def add_points_to_map(self, map_path, points, output_path):
        """
        OpenCV로 맵 이미지에 점 추가
        - points: [(x1, y1), (x2, y2), ...] 점 리스트
        - output_path: 수정된 맵 저장 경로
        """
        try:
            # 맵 로드
            map_image = cv2.imread(map_path, cv2.IMREAD_COLOR)
            if map_image is None:
                self.get_logger().error(f"Failed to load map image from: {map_path}")
                return

            # 점 추가
            for point in points:
                cv2.circle(map_image, point, radius=5, color=(0, 255, 255), thickness=-1)  # 노란색 점

            # 수정된 맵 저장
            cv2.imwrite(output_path, map_image)
            self.get_logger().info(f"Modified map saved to: {output_path}")
        except Exception as e:
            self.get_logger().error(f"Error adding points to map: {e}")



def main(args=None):
    rclpy.init(args=args)
    node = TurtlebotExplorerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # ctrl + c
        node.get_logger().info("KeyboardInterrupt: shutting down node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
