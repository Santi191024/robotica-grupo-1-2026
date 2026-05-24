import rclpy
from rclpy.node import Node
import pandas as pd

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


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

        data = pd.read_csv('trayectoria_scara_gazebo.csv', header=None)

        msg = JointTrajectory()
        msg.joint_names = ['joint_1', 'joint_2', 'joint_3']

        for i in range(len(data)):
            point = JointTrajectoryPoint()

            point.positions = data.iloc[i, 1:4].tolist()
            point.velocities = data.iloc[i, 4:7].tolist()

            point.time_from_start.sec = int(data.iloc[i, 0])
            point.time_from_start.nanosec = int((data.iloc[i, 0] % 1) * 1e9)

            msg.points.append(point)

        self.publisher.publish(msg)

        self.get_logger().info('Trayectoria enviada 🚀')

        self.timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryFromCSV()
    rclpy.spin(node)
    rclpy.shutdown()