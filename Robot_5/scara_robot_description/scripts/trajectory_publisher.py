#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from ament_index_python.packages import get_package_share_directory
import csv
import os
import serial
import time


class TrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('trajectory_publisher')

        self.pub_q1 = self.create_publisher(Float64, '/Union_1_joint/cmd_pos', 10)
        self.pub_q2 = self.create_publisher(Float64, '/union_2_joint/cmd_pos', 10)
        self.pub_q3 = self.create_publisher(Float64, '/prismatic_joint/cmd_pos', 10)

        # ── Serial ESP32 ───────────────────────────────────────
        try:
            self.esp = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
            time.sleep(2)  # esperar boot ESP32
            self.get_logger().info('ESP32 conectada en /dev/ttyUSB0')
        except Exception as e:
            self.get_logger().error(f'No se pudo abrir serial: {e}')
            self.esp = None

        # ── Cargar CSV ─────────────────────────────────────────
        csv_path = os.path.join(
            get_package_share_directory('scara_robot_description'),
            'trayectorias', '00_trayectoria_ROS.csv'
        )
        self.rows = []
        with open(csv_path, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                self.rows.append(row)

        self.index        = 0
        self.total        = len(self.rows)
        self.home_counter = 0
        self.homed        = False

        # HOME en radianes (para Gazebo/ROS)
        self.q1_home = 3.753
        self.q2_home = -2.360
        self.q3_home = 0.0

        self.get_logger().info(f'CSV cargado: {self.total} puntos')
        self.get_logger().info('Enviando HOME a ESP32 y Gazebo...')

        # Mandar home físico a la ESP32
        self._enviar_esp('home')

        self.timer = self.create_timer(0.01, self.timer_callback)

    # ── Envío serial a ESP32 ───────────────────────────────────
    def _enviar_esp(self, cmd: str):
        if self.esp and self.esp.is_open:
            try:
                self.esp.write((cmd + '\n').encode())
                # Leer respuesta (no bloqueante)
                time.sleep(0.005)
                while self.esp.in_waiting:
                    resp = self.esp.readline().decode(errors='ignore').strip()
                    if resp:
                        self.get_logger().info(f'ESP32 << {resp}')
            except Exception as e:
                self.get_logger().warn(f'Error serial: {e}')

    # ── Publicar en Gazebo/ROS ─────────────────────────────────
    def publish_joints(self, q1, q2, q3):
        m1 = Float64(); m1.data = q1; self.pub_q1.publish(m1)
        m2 = Float64(); m2.data = q2; self.pub_q2.publish(m2)
        m3 = Float64(); m3.data = q3; self.pub_q3.publish(m3)

    # ── Timer 100Hz ───────────────────────────────────────────
    def timer_callback(self):

        # ── Fase 0: HOME (2 segundos = 200 ticks a 100Hz) ─────
        if not self.homed:
            self.publish_joints(self.q1_home, self.q2_home, self.q3_home)
            self.home_counter += 1
            if self.home_counter >= 200:
                self.homed = True
                self.get_logger().info('HOME listo — iniciando trayectoria...')
            return

        # ── Fase 1: trayectoria CSV ────────────────────────────
        if self.index >= self.total:
            self.get_logger().info('Trayectoria completada.')
            self.timer.cancel()
            return

        row = self.rows[self.index]
        q1  = float(row['joint1_pos'])
        q2  = float(row['joint2_pos'])
        q3  = float(row['joint3_pos']) / 100.0   # cm → m

        # 1. Publicar en Gazebo / ROS
        self.publish_joints(q1, q2, q3)

        # 2. Enviar a ESP32 física
        # q1: rad → deg
        # q2: rad → deg, invertido porque el motor gira en sentido contrario
        # q3: por ahora siempre 0
        g1  = round(q1 * 57.2958, 3)    # rad → deg
        g2  = round(q2 * -57.2958, 3)   # rad → deg (invertido)
        ms3 = 0                          # prismática deshabilitada por ahora

        cmd = f'Q:{g1:.3f}:{g2:.3f}:{ms3}'
        self._enviar_esp(cmd)

        # Log cada 100 puntos
        if self.index % 100 == 0:
            self.get_logger().info(
                f'[Seg {row["segment"]}] t={float(row["time_s"]):.2f}s | '
                f'q1={q1:.3f}rad ({g1}deg) q2={q2:.3f}rad ({g2}deg) q3={q3*100:.2f}cm (deshabilitada)'
            )

        self.index += 1


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()