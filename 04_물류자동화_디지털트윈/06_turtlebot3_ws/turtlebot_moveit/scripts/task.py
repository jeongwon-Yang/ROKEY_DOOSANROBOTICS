#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlebot_cosmo_interface.srv import MoveitControl
from geometry_msgs.msg import Pose, PoseArray
from srv_call_test import TurtlebotArmClient
from aruco_move_test import ArucoMarkerMoveController
import time

def main(args=None):
    rclpy.init(args=args)
    arm_client = TurtlebotArmClient()
    move_client = ArucoMarkerMoveController()

    print ("task start!")


    # ## gripper control
    response = arm_client.send_request(2, "close")
    arm_client.get_logger().info(f'Response: {response.response}')

    response = arm_client.send_request(9, "")
    arm_client.get_logger().info(f'Response: {response.response}')
    

    print ("task end!")
    rclpy.shutdown()

if __name__ == '__main__':
    main()