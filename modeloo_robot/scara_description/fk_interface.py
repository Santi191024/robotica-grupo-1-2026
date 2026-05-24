#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import tkinter as tk
import math

class FKInterface(Node):
    def __init__(self):
        super().__init__('fk_interface_node')
        self.publisher = self.create_publisher(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            10
        )

    def send_command(self, joints):
        msg = JointTrajectory()
        msg.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4']

        point = JointTrajectoryPoint()
        point.positions = joints
        point.time_from_start.sec = 2  # 2 segundos para alcanzar la posición

        msg.points.append(point)
        self.publisher.publish(msg)
        self.get_logger().info(f"Comando enviado: {joints}")


def main():
    rclpy.init()

    node = FKInterface()

    # ---- Interfaz gráfica ----
    root = tk.Tk()
    root.title("Interfaz SCARA - Cinemática Directa")

    labels = ["Joint 1 (rad)", "Joint 2 (rad)", "Joint 3 (m)", "Joint 4 (rad)"]
    entries = []

    for i, text in enumerate(labels):
        tk.Label(root, text=text).grid(row=i, column=0)
        entry = tk.Entry(root)
        entry.grid(row=i, column=1)
        entries.append(entry)

    output_label = tk.Label(root, text="Posición efector: (x,y,z)")
    output_label.grid(row=5, column=0, columnspan=2)

    def compute_and_send():
        try:
            # Leer valores de articulaciones
            j1 = float(entries[0].get())
            j2 = float(entries[1].get())
            j3 = float(entries[2].get())
            j4 = float(entries[3].get())

            # Parámetros de tu SCARA (ejemplo: longitudes)
            l1, l2 = 0.3, 0.2  # en metros

            # Cinemática directa (simplificada)
            x = l1 * math.cos(j1) + l2 * math.cos(j1 + j2)
            y = l1 * math.sin(j1) + l2 * math.sin(j1 + j2)
            z = j3  # articulación prismática

            output_label.config(text=f"Posición efector: ({x:.3f}, {y:.3f}, {z:.3f})")

            # Mandar comando al robot
            node.send_command([j1, j2, j3, j4])

        except ValueError:
            output_label.config(text="⚠️ Ingresa valores numéricos")

    send_button = tk.Button(root, text="Mover Robot", command=compute_and_send)
    send_button.grid(row=6, column=0, columnspan=2)

    # Correr GUI en paralelo con ROS2
    def tk_loop():
        root.update()
        rclpy.spin_once(node, timeout_sec=0.1)
        root.after(100, tk_loop)

    tk_loop()
    root.mainloop()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()