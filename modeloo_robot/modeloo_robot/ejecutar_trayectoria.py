#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import pandas as pd
import os

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from ament_index_python.packages import get_package_share_directory


class TrajectoryFromCSV(Node):

    def __init__(self):
        super().__init__('trajectory_from_csv')

        self.publisher = self.create_publisher(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            10
        )

        self.timer = self.create_timer(3.0, self.send_trajectory)

    def send_trajectory(self):

        self.get_logger().info('Leyendo CSV...')

        # 🔥 RUTA CORRECTA EN ROS2
        pkg_path = get_package_share_directory('modeloo_robot')
        csv_path = os.path.join(pkg_path, 'data', 'trayectoria_scara_gazebo.csv')

        data = pd.read_csv(csv_path, header=None)

        msg = JointTrajectory()
        msg.joint_names = ['joint_1', 'joint_2', 'joint_3']

        for i in range(len(data)):
            point = JointTrajectoryPoint()

            point.positions = data.iloc[i, 1:4].tolist()
            point.velocities = data.iloc[i, 4:7].tolist()

            t = float(data.iloc[i, 0])
            point.time_from_start.sec = int(t)
            point.time_from_start.nanosec = int((t - int(t)) * 1e9)

            msg.points.append(point)

        self.publisher.publish(msg)

        self.get_logger().info('Trayectoria enviada 🚀')

        self.timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryFromCSV()
    rclpy.spin(node)
    rclpy.shutdown()
