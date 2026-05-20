#!/usr/bin/env python3
"""
Control Cinematico - Robot SCARA (RRP)
Incluye control de Gripper, feedback del eje Z real y corrección Jacobiana.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Bool
from ament_index_python.packages import get_package_share_directory
import numpy as np
import csv, os, serial, time, threading, math

# ── Parámetros del robot ────────────────────────────────────────
L1  = 0.13
L2  = 0.13
KP  = np.diag([6.0, 6.0, 2.0])
DT  = 0.02
DLS_LAMBDA  = 0.005

Q1_HOME_RAD    =  3.75357508226648
Q2_HOME_RAD    = -2.35912515630462
Q3_HOME_CM     = -15.46  # HOME ABSOLUTO EN Z
OFFSET_URDF_Q1 = math.radians(35.0)

class KinematicController(Node):
    def __init__(self):
        super().__init__('kinematic_controller')

        self.pub_q1 = self.create_publisher(Float64, '/Union_1_joint/cmd_pos', 10)
        self.pub_q2 = self.create_publisher(Float64, '/union_2_joint/cmd_pos', 10)
        self.pub_q3 = self.create_publisher(Float64, '/prismatic_joint/cmd_pos', 10)

        self.pub_completo = self.create_publisher(Bool, '/ensamble_3_completo', 10)
        self.create_subscription(Bool, '/orden_ensamble_3', self.cb_orden, 10)
        self.create_subscription(Bool, '/paro_emergencia',  self.cb_paro,  10)
        self.create_subscription(Bool, '/zona_segura',      self.cb_zona,  10)

        self.serial_lock = threading.Lock()
        try:
            self.esp = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1)
            time.sleep(2)
            self.get_logger().info('ESP32 conectada en /dev/ttyUSB0')
        except Exception as e:
            self.get_logger().error(f'Serial: {e}')
            self.esp = None

        self.q_actual = np.array([Q1_HOME_RAD, Q2_HOME_RAD, Q3_HOME_CM / 100.0])
        self.fb_lock = threading.Lock()
        self.fb_recibido = False
        self.home_esp_ok = False
        self.last_gripper_pos = -1  # Evita enviar el comando G repetidamente

        threading.Thread(target=self._leer_serial, daemon=True).start()

        try:
            csv_path = os.path.join(
                get_package_share_directory('scara_robot_description'),
                'trayectorias', '00_trayectoria_ROS.csv'
            )
            self.rows = []
            with open(csv_path, 'r') as f:
                for row in csv.DictReader(f):
                    self.rows.append(row)
            self.total = len(self.rows)
            self.get_logger().info(f'CSV cargado: {self.total} puntos')
        except Exception as e:
            self.get_logger().error(f'Error cargando CSV: {e}')
            self.rows = []
            self.total = 0

        self.orden_activa = False
        self.ejecutando   = False
        self.completo     = False
        self.homed        = False
        self.home_counter = 0
        self.index        = 0

        self._pub_completo(False)
        self.get_logger().info('Nodo listo — esperando /orden_ensamble_3')
        self.create_timer(DT, self.timer_cb)

    @staticmethod
    def fk(q):
        q1, q2, q3 = q
        x = L1 * np.cos(q1) + L2 * np.cos(q1 + q2)
        y = L1 * np.sin(q1) + L2 * np.sin(q1 + q2)
        return np.array([x, y, q3])

    @staticmethod
    def jacobiano(q):
        q1, q2, _ = q
        s1  = np.sin(q1);       c1  = np.cos(q1)
        s12 = np.sin(q1 + q2);  c12 = np.cos(q1 + q2)
        return np.array([
            [-(L1*s1 + L2*s12), -L2*s12, 0.0],
            [ (L1*c1 + L2*c12),  L2*c12, 0.0],
            [              0.0,     0.0,  1.0]
        ])

    @staticmethod
    def J_inv_dls(J, lam=DLS_LAMBDA):
        return J.T @ np.linalg.inv(J @ J.T + lam**2 * np.eye(3))

    def control(self, q_actual, q_ref, x_ref, xdot_ref):
        x_actual = self.fk(q_actual)
        e        = x_ref - x_actual
        J        = self.jacobiano(q_actual)
        J_inv    = self.J_inv_dls(J)
        qdot     = J_inv @ (KP @ e + xdot_ref)
        q_cmd    = q_ref + qdot * DT 
        return q_cmd, e

    def _leer_serial(self):
        while True:
            if not (self.esp and self.esp.is_open):
                time.sleep(0.1)
                continue
            try:
                with self.serial_lock:
                    raw = self.esp.readline()
                line = raw.decode(errors='ignore').strip()
                if not line: continue

                # LEER LOS TRES ENCODERS (M1, M2, Z)
                if line.startswith('FB:'):
                    p = line.split(':')
                    if len(p) >= 4:
                        with self.fb_lock:
                            self.q_actual[0] = np.radians(float(p[1]))
                            self.q_actual[1] = -np.radians(float(p[2]))
                            self.q_actual[2] = float(p[3]) / 100.0  # Pasar de CM a METROS
                            self.fb_recibido = True

                elif line.startswith('HOME:'):
                    p = line.split(':')
                    if len(p) >= 4:
                        with self.fb_lock:
                            self.q_actual[0] = np.radians(float(p[1]))
                            self.q_actual[1] = -np.radians(float(p[2]))
                            self.q_actual[2] = float(p[3]) / 100.0  # El -15.46 en metros
                            self.fb_recibido = True
                        self.home_esp_ok = True
                        self.get_logger().info('ESP32: HOME físico confirmado ✅')

                elif line.startswith('STATUS:') or line.startswith('OK:') or line.startswith('TEL|'):
                    pass # Ocultar logs recurrentes para limpieza
                else:
                    self.get_logger().info(f'ESP32: {line}')

            except Exception as e:
                time.sleep(0.01)

    def _enviar(self, cmd: str):
        if self.esp and self.esp.is_open:
            try:
                with self.serial_lock:
                    self.esp.write((cmd + '\n').encode())
            except Exception as e:
                self.get_logger().warn(f'Serial write: {e}')

    def _pub_joints(self, q):
        m = Float64()
        m.data = float(q[0]) - OFFSET_URDF_Q1; self.pub_q1.publish(m)
        m.data = float(q[1]);                   self.pub_q2.publish(m)
        m.data = float(q[2]);                   self.pub_q3.publish(m)

    def _pub_completo(self, v: bool):
        m = Bool(); m.data = v; self.pub_completo.publish(m)

    def cb_orden(self, msg: Bool):
        if msg.data and not self.orden_activa:
            self.orden_activa = True; self.ejecutando = True; self.completo = False
            self.homed = False; self.home_esp_ok = False; self.index = 0
            self._pub_completo(False)
            self._enviar('home')
        elif not msg.data and self.orden_activa:
            self.orden_activa = False; self.ejecutando = False
            self._pub_completo(False)

    def cb_paro(self, msg: Bool):
        if msg.data: self._parar()

    def cb_zona(self, msg: Bool):
        if not msg.data: self._parar()

    def _parar(self):
        self._enviar('home')
        self.orden_activa = False; self.ejecutando = False; self.completo = False
        self._pub_completo(False)

    def timer_cb(self):
        if not self.ejecutando or self.total == 0: return

        if not self.homed:
            self._pub_joints(np.array([Q1_HOME_RAD, Q2_HOME_RAD, Q3_HOME_CM / 100.0]))
            self.home_counter += 1
            if self.home_esp_ok or self.home_counter >= 1000:
                self.get_logger().info('HOME OK → arrancando CSV')
                self.homed = True
            return

        if self.index >= self.total:
            if not self.completo:
                self.completo = True; self.ejecutando = False
                self._pub_completo(True)
                self.get_logger().info('ensamble_3_completo=1')
            return

        row = self.rows[self.index]
        q_ref = np.array([float(row['joint1_pos']), float(row['joint2_pos']), float(row['joint3_pos']) / 100.0])
        x_ref = self.fk(q_ref)

        qdot_ref = np.array([
            float(row['joint1_vel']), float(row['joint2_vel']), float(row['joint3_vel']) / 100.0
        ]) if 'joint1_vel' in row else np.zeros(3)

        xdot_ref = self.jacobiano(q_ref) @ qdot_ref

        with self.fb_lock: q = self.q_actual.copy()
        if not self.fb_recibido: q = q_ref.copy()

        q_cmd, e = self.control(q, q_ref, x_ref, xdot_ref)

        # === COMANDO DEL GRIPPER ===
        # Leemos la nueva columna 'gripper_pos' que exportó MATLAB
        gripper_cmd = int(float(row.get('gripper_pos', 90)))
        if gripper_cmd != self.last_gripper_pos:
            self._enviar(f'G:{gripper_cmd}')
            self.last_gripper_pos = gripper_cmd

        self._pub_joints(q_cmd)

        g1  =  np.degrees(q_cmd[0])
        g2  = -np.degrees(q_cmd[1])
        cm3 =  q_cmd[2] * 100.0
        self._enviar(f'Q:{g1:.3f}:{g2:.3f}:{cm3:.2f}')

        if self.index % 50 == 0:
            norm_e = np.linalg.norm(e) * 100
            self.get_logger().info(f'[{self.index}/{self.total}] |e|={norm_e:.2f}cm '
                                   f'q=[{np.degrees(q[0]):.1f}°, {np.degrees(q[1]):.1f}°, {q[2]*100:.1f}cm]')

        self.index += 1

def main(args=None):
    rclpy.init(args=args)
    node = KinematicController()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__': main()
