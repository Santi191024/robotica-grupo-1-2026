import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import csv
import time
import os

class TrajectoryPublisher(Node):

    def __init__(self):
        super().__init__('trajectory_publisher')
        self.publisher_ = self.create_publisher(JointState, '/robot/joint_states', 10)  # Topic de Gazebo para joint_states
        self.get_logger().info("Iniciando el publicador de trayectorias de juntas")

        # Configurar el timer para publicar cada 20 ms (0.02 segundos)
        self.timer = self.create_timer(0.02, self.publish_trajectory)
        self.trajectory = self.load_all_trajectories_from_csvs("/home/schneider/trayectoriaslaboratorio")  # Ruta de tus archivos CSV
        self.index = 0

    def load_all_trajectories_from_csvs(self, folder_path):
        all_trajectories = []
        for filename in os.listdir(folder_path):
            if filename.endswith('.csv'):
                file_path = os.path.join(folder_path, filename)
                all_trajectories.append(self.load_trajectory_from_csv(file_path))
        return all_trajectories

    def load_trajectory_from_csv(self, filename):
        trajectory = []
        with open(filename, 'r') as file:
            reader = csv.reader(file)
            next(reader)  # Salta la cabecera si la tiene
            for row in reader:
                # Leer los ángulos de las juntas
                angles = [float(angle) for angle in row]  # Ángulos de cada junta en cada fila
                trajectory.append(angles)
        return trajectory

    def publish_trajectory(self):
        if self.index < len(self.trajectory):
            joint_angles = self.trajectory[self.index]  # Obtener los ángulos de las juntas del archivo CSV actual
            joint_state_msg = JointState()
            joint_state_msg.header.stamp = self.get_clock().now().to_msg()
            joint_state_msg.name = ["JOINT1", "JOINT2", "JOINT3"]  # Nombres de tus juntas
            joint_state_msg.position = joint_angles  # Ángulos de las juntas

            self.publisher_.publish(joint_state_msg)
            self.get_logger().info(f"Publicando trayectorias de juntas: {joint_angles}")
            self.index += 1
        else:
            self.get_logger().info("Trayectoria completada")
            self.destroy_timer(self.timer)  # Detener el timer cuando se hayan publicado todas las trayectorias

def main(args=None):
    rclpy.init(args=args)

    trajectory_publisher = TrajectoryPublisher()

    rclpy.spin(trajectory_publisher)

    trajectory_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
