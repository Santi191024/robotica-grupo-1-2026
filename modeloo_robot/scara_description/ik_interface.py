#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import tkinter as tk
import math


class IKInterface(Node):
    def __init__(self):
        super().__init__('ik_interface_node')
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
        point.time_from_start.sec = 2

        msg.points.append(point)
        self.publisher.publish(msg)
        self.get_logger().info(f"Comando enviado: {joints}")


def main():
    rclpy.init()
    node = IKInterface()

    # ---- Interfaz gráfica ----
    root = tk.Tk()
    root.title("Interfaz SCARA - Cinemática Inversa")

    labels = ["X (m)", "Y (m)", "Z (m)", "Orientación (rad)"]
    entries = []

    for i, text in enumerate(labels):
        tk.Label(root, text=text).grid(row=i, column=0)
        entry = tk.Entry(root)
        entry.grid(row=i, column=1)
        entries.append(entry)

    output_label = tk.Label(root, text="Articulaciones: (j1,j2,j3,j4)")
    output_label.grid(row=5, column=0, columnspan=2)

    def compute_and_send():
        try:
            # Leer valores de entrada
            x = float(entries[0].get())
            y = float(entries[1].get())
            z = float(entries[2].get())
            phi = float(entries[3].get())  # orientación de la muñeca

            # ---- Parámetros del robot SCARA ----
            l1, l2 = 0.3, 0.2  # longitudes de eslabones en metros

            # ---- Cinemática Inversa ----
            # J1: ángulo base
            j1 = math.atan2(y, x)

            # Distancia al punto en XY
            r = math.sqrt(x**2 + y**2)

            # Ley de cosenos para J2
            cos_j2 = (r**2 - l1**2 - l2**2) / (2 * l1 * l2)
            if cos_j2 < -1 or cos_j2 > 1:
                output_label.config(text="⚠️ Posición fuera de alcance")
                return
            j2 = math.acos(cos_j2)

            # J1 refinado (usando geometría)
            k1 = l1 + l2 * math.cos(j2)
            k2 = l2 * math.sin(j2)
            j1 = math.atan2(y, x) - math.atan2(k2, k1)

            # J3: prismatic (z)
            j3 = z

            # J4: orientación (simplemente phi - (j1+j2))
            j4 = phi - (j1 + j2)

            joints = [j1, j2, j3, j4]
            output_label.config(text=f"Articulaciones: {joints}")

            # Mandar al robot
            node.send_command(joints)

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

