#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Bool
from ament_index_python.packages import get_package_share_directory
import csv
import os
import serial
import time


class TrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('trajectory_publisher')

        # ── Publishers Gazebo/ROS ──────────────────────────────
        self.pub_q1 = self.create_publisher(Float64, '/Union_1_joint/cmd_pos', 10)
        self.pub_q2 = self.create_publisher(Float64, '/union_2_joint/cmd_pos', 10)
        self.pub_q3 = self.create_publisher(Float64, '/prismatic_joint/cmd_pos', 10)

        # ── SALIDA al supervisor: ensamble_3_completo ──────────
        self.pub_completo = self.create_publisher(
            Bool, '/ensamble_3_completo', 10)

        # ── ENTRADAS del supervisor ────────────────────────────
        # Activación: supervisor en S7 → orden_ensamble_3 = 1
        self.sub_orden = self.create_subscription(
            Bool, '/orden_ensamble_3', self.cb_orden, 10)

        # Seguridad global: paro de emergencia
        self.sub_paro = self.create_subscription(
            Bool, '/paro_emergencia', self.cb_paro, 10)

        # Seguridad global: zona segura
        self.sub_zona = self.create_subscription(
            Bool, '/zona_segura', self.cb_zona, 10)

        # ── Serial ESP32 ───────────────────────────────────────
        try:
            self.esp = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
            time.sleep(2)
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

        self.total = len(self.rows)
        self.get_logger().info(f'CSV cargado: {self.total} puntos')

        # ── Estado interno ─────────────────────────────────────
        self.orden_activa = False   # True mientras supervisor está en S7
        self.ejecutando   = False   # True mientras corre la trayectoria
        self.completo     = False   # True cuando terminó, hasta ACK del supervisor
        self.homed        = False
        self.home_counter = 0
        self.index        = 0

        # HOME en radianes (Gazebo)
        self.q1_home = 3.753
        self.q2_home = -2.360
        self.q3_home = 0.0

        # Publicar completo=False al arrancar (estado limpio)
        self._publicar_completo(False)

        self.get_logger().info('Nodo listo — esperando /orden_ensamble_3...')
        self.timer = self.create_timer(0.01, self.timer_callback)

    # ──────────────────────────────────────────────────────────
    # CALLBACKS FSM
    # ──────────────────────────────────────────────────────────

    def cb_orden(self, msg: Bool):
        # Flanco de SUBIDA: supervisor entra a S7
        if msg.data and not self.orden_activa:
            self.get_logger().info('orden_ensamble_3 = 1 → iniciando HOME + trayectoria')
            self.orden_activa = True
            self.ejecutando   = True
            self.completo     = False
            self.homed        = False
            self.home_counter = 0
            self.index        = 0
            self._publicar_completo(False)
            self._enviar_esp('home')

        # Flanco de BAJADA: supervisor salió de S7 (leyó ensamble_3_completo)
        elif not msg.data and self.orden_activa:
            self.get_logger().info('orden_ensamble_3 = 0 → reseteando estado')
            self.orden_activa = False
            self.ejecutando   = False
            self.completo     = False
            self._publicar_completo(False)

    def cb_paro(self, msg: Bool):
        # paro_emergencia = 1 → parar todo inmediatamente (FSM va a S0)
        if msg.data:
            self.get_logger().warn('PARO DE EMERGENCIA — deteniendo robot')
            self._parar_todo()

    def cb_zona(self, msg: Bool):
        # zona_segura = 0 → parar todo inmediatamente (FSM va a S0)
        if not msg.data:
            self.get_logger().warn('ZONA NO SEGURA — deteniendo robot')
            self._parar_todo()

    def _parar_todo(self):
        self._enviar_esp('home')
        self.orden_activa = False
        self.ejecutando   = False
        self.completo     = False
        self._publicar_completo(False)

    # ──────────────────────────────────────────────────────────
    # PUBLICAR ensamble_3_completo
    # ──────────────────────────────────────────────────────────
    def _publicar_completo(self, valor: bool):
        msg = Bool()
        msg.data = valor
        self.pub_completo.publish(msg)

    # ── Envío serial ESP32 ─────────────────────────────────────
    def _enviar_esp(self, cmd: str):
        if self.esp and self.esp.is_open:
            try:
                self.esp.write((cmd + '\n').encode())
                time.sleep(0.005)
                while self.esp.in_waiting:
                    resp = self.esp.readline().decode(errors='ignore').strip()
                    if resp:
                        self.get_logger().info(f'ESP32 << {resp}')
            except Exception as e:
                self.get_logger().warn(f'Error serial: {e}')

    # ── Publicar joints Gazebo ─────────────────────────────────
    def publish_joints(self, q1, q2, q3):
        m1 = Float64(); m1.data = q1; self.pub_q1.publish(m1)
        m2 = Float64(); m2.data = q2; self.pub_q2.publish(m2)
        m3 = Float64(); m3.data = q3; self.pub_q3.publish(m3)

    # ──────────────────────────────────────────────────────────
    # TIMER 100Hz
    # ──────────────────────────────────────────────────────────
    def timer_callback(self):

        # Sin orden activa → no hacer nada
        if not self.ejecutando:
            return

        # ── Fase 0: HOME (2s = 200 ticks) ─────────────────────
        if not self.homed:
            self.publish_joints(self.q1_home, self.q2_home, self.q3_home)
            self.home_counter += 1
            if self.home_counter >= 200:
                self.homed = True
                self.get_logger().info('HOME listo — ejecutando trayectoria...')
            return

        # ── Fase 1: trayectoria terminada ─────────────────────
        if self.index >= self.total:
            if not self.completo:
                self.completo   = True
                self.ejecutando = False
                self._publicar_completo(True)   # → supervisor transita S7→S8
                self.get_logger().info('ensamble_3_completo = 1 → esperando ACK supervisor')
            return

        # ── Fase 2: ejecutar punto del CSV ────────────────────
        row = self.rows[self.index]
        q1  = float(row['joint1_pos'])
        q2  = float(row['joint2_pos'])
        q3  = float(row['joint3_pos']) / 100.0   # cm → m

        # Publicar en Gazebo
        self.publish_joints(q1, q2, q3)

        # Enviar a ESP32: rad → grados
        g1  = round(q1 *  57.2958, 3)
        g2  = round(q2 * -57.2958, 3)   # invertido por sentido del motor
        cmd = f'Q:{g1:.3f}:{g2:.3f}:0'
        self._enviar_esp(cmd)

        if self.index % 100 == 0:
            self.get_logger().info(
                f'[Seg {row["segment"]}] t={float(row["time_s"]):.2f}s | '
                f'q1={q1:.3f}rad ({g1}deg) | q2={q2:.3f}rad ({g2}deg)'
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