#!/usr/bin/env python3
"""
Control Cinematico - Robot SCARA (RRP)
Ley de control:  qdot = J_A^{-1}(q) * (Kp*e + xdot_ref)
                 e    = x_ref - f(q_actual)
                 q_cmd = q_actual + qdot * dt

PROTOCOLO SERIAL:
  ROS  -> ESP32:  Q:deg1:deg2:ms3   (100 Hz)
                  home
                  status
  ESP32 -> ROS:   FB:deg1:deg2      (50 Hz)  <- cierra el lazo
                  OK:deg1:deg2:ms3  (ack)
                  HOME:deg1:deg2:0  (confirmación tras home físico)
                  STATUS:deg1:deg2:0
                  TEL|...           (debug)

OFFSET HOME (primera fila del CSV):
  q1_home = 3.75357 rad = 215.064°
  q2_home = -2.35913 rad = -135.168°  (ESP32 la envía como +135.168° — invertida)
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Bool
from ament_index_python.packages import get_package_share_directory
import numpy as np
import csv, os, serial, time, threading, math


# ── Parámetros del robot SCARA ─────────────────────────────────
L1  = 0.13          # m
L2  = 0.13          # m
KP  = np.diag([5.0, 5.0, 1.5])
DT  = 0.01          # s  = 100 Hz
EPS = 1e-6

# Offset HOME — primera fila del CSV
Q1_HOME_RAD =  3.75357508226648    # rad
Q2_HOME_RAD = -2.35912515630462    # rad
Q3_HOME_CM  =  0.0                 # cm
OFFSET_URDF_Q1 = math.radians(35.0)   # solo para publicar en Gazebo

# Conversión prismático
CM_POR_SEGUNDO = 2.0
MS_POR_CM      = 1000.0 / CM_POR_SEGUNDO   # = 500 ms/cm

# Tolerancia HOME
TOL_HOME_RAD = math.radians(1.5)   # 1.5°


class KinematicController(Node):
    def __init__(self):
        super().__init__('kinematic_controller')

        # ── Publishers Gazebo ──────────────────────────────────
        self.pub_q1 = self.create_publisher(Float64, '/Union_1_joint/cmd_pos', 10)
        self.pub_q2 = self.create_publisher(Float64, '/union_2_joint/cmd_pos', 10)
        self.pub_q3 = self.create_publisher(Float64, '/prismatic_joint/cmd_pos', 10)

        # ── FSM I/O ────────────────────────────────────────────
        self.pub_completo = self.create_publisher(Bool, '/ensamble_3_completo', 10)
        self.create_subscription(Bool, '/orden_ensamble_3', self.cb_orden, 10)
        self.create_subscription(Bool, '/paro_emergencia',  self.cb_paro,  10)
        self.create_subscription(Bool, '/zona_segura',      self.cb_zona,  10)

        # ── Serial ESP32 ───────────────────────────────────────
        try:
            self.esp = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1)
            time.sleep(2)
            self.get_logger().info('ESP32 conectada en /dev/ttyUSB0')
        except Exception as e:
            self.get_logger().error(f'Serial: {e}')
            self.esp = None

        # ── Estado articular real (feedback encoder ESP32) ─────
        # Inicializado en HOME del CSV para que el primer tick no salte
        self.q_actual    = np.array([Q1_HOME_RAD, Q2_HOME_RAD, Q3_HOME_CM / 100.0])
        self.fb_lock     = threading.Lock()
        self.fb_recibido = False
        self.home_esp_ok = False   # True cuando ESP32 confirma HOME físico

        threading.Thread(target=self._leer_serial, daemon=True).start()

        # ── Cargar CSV ─────────────────────────────────────────
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

        # ── Estado FSM ─────────────────────────────────────────
        self.orden_activa = False
        self.ejecutando   = False
        self.completo     = False
        self.homed        = False
        self.home_counter = 0
        self.index        = 0

        self._pub_completo(False)
        self.get_logger().info('Nodo listo — esperando /orden_ensamble_3')
        self.create_timer(DT, self.timer_cb)

    # ══════════════════════════════════════════════════════════
    # CINEMÁTICA DIRECTA
    # ══════════════════════════════════════════════════════════
    @staticmethod
    def fk(q):
        q1, q2, q3 = q
        x = L1 * np.cos(q1) + L2 * np.cos(q1 + q2)
        y = L1 * np.sin(q1) + L2 * np.sin(q1 + q2)
        return np.array([x, y, q3])

    # ══════════════════════════════════════════════════════════
    # JACOBIANO ANALÍTICO 3x3
    # ══════════════════════════════════════════════════════════
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

    # ══════════════════════════════════════════════════════════
    # LEY DE CONTROL
    # ══════════════════════════════════════════════════════════
    def control(self, q, x_ref, xdot_ref):
        x_actual = self.fk(q)
        e        = x_ref - x_actual
        J        = self.jacobiano(q)
        det      = np.linalg.det(J)
        if abs(det) < EPS:
            self.get_logger().warn(f'Singularidad: det={det:.2e} — usando pinv')
            J_inv = np.linalg.pinv(J)
        else:
            J_inv = np.linalg.inv(J)
        qdot  = J_inv @ (KP @ e + xdot_ref)
        q_cmd = q + qdot * DT
        return q_cmd, e

    # ══════════════════════════════════════════════════════════
    # HILO SERIAL — parsea mensajes ESP32
    # ══════════════════════════════════════════════════════════
    def _leer_serial(self):
        while True:
            if not (self.esp and self.esp.is_open):
                time.sleep(0.1); continue
            try:
                line = self.esp.readline().decode(errors='ignore').strip()
                if not line: continue

                # FB:deg1:deg2  →  actualizar q_actual (50 Hz)
                if line.startswith('FB:'):
                    p = line.split(':')
                    if len(p) == 3:
                        d1 = float(p[1])
                        d2 = float(p[2])
                        r1 =  np.radians(d1)
                        r2 = -np.radians(d2)   # re-invertir igual que la ESP
                        with self.fb_lock:
                            self.q_actual[0] = r1
                            self.q_actual[1] = r2
                            self.fb_recibido = True

                # HOME:deg1:deg2:0  →  ESP32 completó home físico
                elif line.startswith('HOME:'):
                    p = line.split(':')
                    if len(p) >= 3:
                        d1 = float(p[1])
                        d2 = float(p[2])
                        r1 =  np.radians(d1)
                        r2 = -np.radians(d2)
                        with self.fb_lock:
                            self.q_actual[0] = r1
                            self.q_actual[1] = r2
                            self.q_actual[2] = 0.0
                            self.fb_recibido = True
                        self.home_esp_ok = True
                        self.get_logger().info('ESP32: HOME físico confirmado ✅')

                elif line.startswith('STATUS:'):
                    self.get_logger().info(f'ESP32 status: {line}')

                elif line.startswith('OK:'):
                    pass   # ack silencioso a 100 Hz

                elif line.startswith('TEL|'):
                    self.get_logger().debug(f'ESP32: {line[4:]}')

                else:
                    self.get_logger().info(f'ESP32: {line}')

            except Exception as e:
                self.get_logger().warn(f'Serial read: {e}')
                time.sleep(0.01)

    # ── Enviar a ESP32 ─────────────────────────────────────────
    def _enviar(self, cmd: str):
        if self.esp and self.esp.is_open:
            try:
                self.esp.write((cmd + '\n').encode())
            except Exception as e:
                self.get_logger().warn(f'Serial write: {e}')

    # ── Publicar joints en Gazebo ──────────────────────────────
    def _pub_joints(self, q):
        m = Float64()
        # q1: restar offset URDF para que Gazebo dibuje bien
        m.data = float(q[0]) - OFFSET_URDF_Q1; self.pub_q1.publish(m)
        m.data = float(q[1]);                   self.pub_q2.publish(m)
        m.data = float(q[2]);                   self.pub_q3.publish(m)

    def _pub_completo(self, v: bool):
        m = Bool(); m.data = v; self.pub_completo.publish(m)

    # ══════════════════════════════════════════════════════════
    # CALLBACKS FSM
    # ══════════════════════════════════════════════════════════
    def cb_orden(self, msg: Bool):
        if msg.data and not self.orden_activa:
            self.get_logger().info('orden_ensamble_3=1 → iniciando')
            self.orden_activa = True
            self.ejecutando   = True
            self.completo     = False
            self.homed        = False
            self.home_esp_ok  = False
            self.home_counter = 0
            self.index        = 0
            self._pub_completo(False)
            self._enviar('home')   # arranca home físico en ESP32
        elif not msg.data and self.orden_activa:
            self.get_logger().info('orden_ensamble_3=0 → reset')
            self.orden_activa = False
            self.ejecutando   = False
            self._pub_completo(False)

    def cb_paro(self, msg: Bool):
        if msg.data:
            self.get_logger().warn('PARO DE EMERGENCIA')
            self._parar()

    def cb_zona(self, msg: Bool):
        if not msg.data:
            self.get_logger().warn('ZONA NO SEGURA — deteniendo')
            self._parar()

    def _parar(self):
        self._enviar('home')
        self.orden_activa = False
        self.ejecutando   = False
        self.completo     = False
        self._pub_completo(False)

    # ══════════════════════════════════════════════════════════
    # TIMER 100 Hz — LAZO PRINCIPAL
    # ══════════════════════════════════════════════════════════
    def timer_cb(self):
        if not self.ejecutando:
            return

        # ── Fase 0: esperar HOME físico de la ESP32 ────────────
        # Publica HOME en Gazebo mientras espera.
        # Avanza cuando ESP32 confirma HOME: o tras 20 s de timeout.
        if not self.homed:
            q_home = np.array([Q1_HOME_RAD, Q2_HOME_RAD, Q3_HOME_CM / 100.0])
            self._pub_joints(q_home)
            self.home_counter += 1

            home_ok = self.home_esp_ok or (self.home_counter >= 2000)  # 20 s máx
            if home_ok:
                if self.home_esp_ok:
                    self.get_logger().info('HOME confirmado por ESP32 ✅ → arrancando CSV')
                else:
                    self.get_logger().warn('HOME timeout 20s → arrancando de todas formas')
                self.homed = True
            return

        # ── Fase 1: trayectoria terminada ─────────────────────
        if self.index >= self.total:
            if not self.completo:
                self.completo   = True
                self.ejecutando = False
                self._pub_completo(True)
                self.get_logger().info('ensamble_3_completo=1')
            return

        # ── Fase 2: control cinemático a 100 Hz ───────────────
        row = self.rows[self.index]

        q_ref = np.array([
            float(row['joint1_pos']),
            float(row['joint2_pos']),
            float(row['joint3_pos']) / 100.0   # cm → m
        ])
        x_ref    = self.fk(q_ref)
        xdot_ref = np.array([
            float(row['joint1_vel']),   # feedforward de velocidad del CSV
            float(row['joint2_vel']),
            float(row['joint3_vel']) / 100.0
        ]) if 'joint1_vel' in row else np.zeros(3)

        with self.fb_lock:
            q = self.q_actual.copy()

        if not self.fb_recibido:
            q = q_ref.copy()   # estimado inicial hasta que llegue el primer FB:

        # ── Ley de control ─────────────────────────────────────
        q_cmd, e = self.control(q, x_ref, xdot_ref)

        # ── Publicar en Gazebo ─────────────────────────────────
        self._pub_joints(q_cmd)

        # ── Enviar a ESP32 ─────────────────────────────────────
        g1  =  np.degrees(q_cmd[0])         # rad → deg (sin invertir)
        g2  = -np.degrees(q_cmd[1])         # rad → deg (invertido, ESP re-invierte)
        dz  =  q_cmd[2] - q[2]             # delta prismático [m]
        ms3 =  int(dz * 100.0 * MS_POR_CM) # m → cm → ms
        self._enviar(f'Q:{g1:.3f}:{g2:.3f}:{ms3}')

        if self.index % 100 == 0:
            norm_e = np.linalg.norm(e) * 100
            self.get_logger().info(
                f'[{self.index}/{self.total}] |e|={norm_e:.2f}cm ' +
                f'q=[{np.degrees(q[0]):.1f}°, {np.degrees(q[1]):.1f}°, {q[2]*100:.1f}cm]'
            )

        self.index += 1


def main(args=None):
    rclpy.init(args=args)
    node = KinematicController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
