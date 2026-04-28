#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import csv
import os

class CsvTrajectoryPlayer(Node):
    def __init__(self):
        super().__init__('csv_trajectory_player')
        
        self.publisher_ = self.create_publisher(
            JointTrajectory, 
            '/arm_controller/joint_trajectory', 
            10)

        # Esperamos 1.5s para que Gazebo cargue bien todo antes de enviar
        self.timer = self.create_timer(1.5, self.enviar_trayectoria)

    def enviar_trayectoria(self):
        self.timer.cancel() 
        
        msg = JointTrajectory()
        msg.joint_names = ['JOINT1', 'JOINT2', 'JOINT3']

        carpeta_base = os.path.expanduser('/home/schneider/prueba/')
        nombres_archivos = [f'Pos{i}.csv' for i in range(1, 22)]

        tiempo_acumulado = 0.0    
        duracion_movimiento = 0.5 # Tiempo de ejecución rápido de los archivos
        tiempo_espera = 2.0       # Pausa entre posiciones
        
        puntos_totales = 0
        primera_posicion_procesada = False # Variable para controlar el arranque

        for nombre_archivo in nombres_archivos:
            ruta_csv = os.path.join(carpeta_base, nombre_archivo)
            
            if not os.path.exists(ruta_csv):
                self.get_logger().warning(f'⚠️ Saltando {nombre_archivo} (No existe)')
                continue

            try:
                with open(ruta_csv, 'r') as f:
                    filas = list(csv.reader(f))
                    if not filas:
                        continue
                    
                    paso_tiempo = duracion_movimiento / len(filas)
                    self.get_logger().info(f'Procesando {nombre_archivo}...')
                    
                    ultimo_punto_pos = [0.0, 0.0, 0.0]

                    for fila in filas:
                        if len(fila) < 3: continue
                        
                        punto_actual = [float(fila[0]), float(fila[1]), float(fila[2])]
                        
                        # --- FASE DE PREPARACIÓN INICIAL (Solo ocurre una vez) ---
                        if not primera_posicion_procesada:
                            self.get_logger().info('🤖 Moviendo suavemente a la posición de inicio (Prep phase)...')
                            
                            # 1. Viaje suave de 3 segundos hacia la pose inicial
                            tiempo_acumulado += 3.0
                            point_prep = JointTrajectoryPoint()
                            point_prep.positions = punto_actual
                            sec = int(tiempo_acumulado)
                            nanosec = int((tiempo_acumulado - sec) * 1e9)
                            point_prep.time_from_start = Duration(sec=sec, nanosec=nanosec)
                            msg.points.append(point_prep)
                            
                            # 2. Pausa de 1 segundo para estabilizar
                            tiempo_acumulado += 1.0
                            point_pausa_prep = JointTrajectoryPoint()
                            point_pausa_prep.positions = punto_actual
                            sec = int(tiempo_acumulado)
                            nanosec = int((tiempo_acumulado - sec) * 1e9)
                            point_pausa_prep.time_from_start = Duration(sec=sec, nanosec=nanosec)
                            msg.points.append(point_pausa_prep)
                            
                            primera_posicion_procesada = True
                        # ---------------------------------------------------------

                        # Ejecución normal y rápida del archivo
                        ultimo_punto_pos = punto_actual
                        point = JointTrajectoryPoint()
                        point.positions = ultimo_punto_pos
                        
                        tiempo_acumulado += paso_tiempo
                        sec = int(tiempo_acumulado)
                        nanosec = int((tiempo_acumulado - sec) * 1e9)
                        point.time_from_start = Duration(sec=sec, nanosec=nanosec)
                        
                        msg.points.append(point)
                        puntos_totales += 1

                    # --- PAUSA DESPUÉS DE CADA ARCHIVO ---
                    tiempo_acumulado += tiempo_espera
                    pausa_point = JointTrajectoryPoint()
                    pausa_point.positions = ultimo_punto_pos 
                    
                    sec_p = int(tiempo_acumulado)
                    nanosec_p = int((tiempo_acumulado - sec_p) * 1e9)
                    pausa_point.time_from_start = Duration(sec=sec_p, nanosec=nanosec_p)
                    
                    msg.points.append(pausa_point)
                    puntos_totales += 1

            except Exception as e:
                self.get_logger().error(f'Error en {nombre_archivo}: {str(e)}')

        if puntos_totales > 0:
            self.get_logger().info(f'🚀 Enviando trayectoria final. Total puntos: {puntos_totales}')
            self.publisher_.publish(msg)
        else:
            self.get_logger().error('No se cargaron puntos.')

def main(args=None):
    rclpy.init(args=args)
    node = CsvTrajectoryPlayer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__':
    main()
