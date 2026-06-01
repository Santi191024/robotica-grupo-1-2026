import sympy as sp
import numpy as np
import serial
import time

# ------------------ CINEMÁTICA DIRECTA ------------------
def cinematica_directa():
    
    theta1, theta2, theta3, L1, a1, a2, a3 = sp.symbols('theta1 theta2 theta3 L1 a1 a2 a3')
    
    A01 = sp.Matrix([
        [sp.cos(theta1), -sp.sin(theta1)*sp.cos(sp.pi/2),  sp.sin(theta1)*sp.sin(sp.pi/2), a1*sp.cos(theta1)],
        [sp.sin(theta1),  sp.cos(sp.pi/2)*sp.cos(theta1), -sp.sin(sp.pi/2)*sp.cos(theta1), a1*sp.sin(theta1)],
        [0,               sp.sin(sp.pi/2),                sp.cos(sp.pi/2),                L1],
        [0,               0,                              0,                              1]
    ])

    A12 = sp.Matrix([
        [sp.cos(theta2), -sp.sin(theta2), 0, a2*sp.cos(theta2)],
        [sp.sin(theta2),  sp.cos(theta2), 0, a2*sp.sin(theta2)],
        [0,               0,              1, 0],
        [0,               0,              0, 1]
    ])

    A23 = sp.Matrix([
        [sp.cos(theta3), -sp.sin(theta3), 0, a3*sp.cos(theta3)],
        [sp.sin(theta3),  sp.cos(theta3), 0, a3*sp.sin(theta3)],
        [0,               0,              1, 0],
        [0,               0,              0, 1]
    ])

    return sp.simplify(A01 * A12 * A23), (theta1, theta2, theta3, L1, a1, a2, a3)


# ------------------ JACOBIANO ------------------
def jacobiano():
    T, variables = cinematica_directa()
    theta1, theta2, theta3, L1, a1, a2, a3 = variables

    pos = T[0:3, 3]
    Jv = pos.jacobian([theta1, theta2, theta3])

    return Jv, variables


# ------------------ CINEMÁTICA INVERSA ------------------
def cinematica_inversa():
    
    Px, Py, Pz, L1, a1, a2, a3 = sp.symbols('Px Py Pz L1 a1 a2 a3')
    
    theta1 = sp.atan2(Py, Px)

    Px_p = sp.sqrt(Px**2 + Py**2)
    R = sp.sqrt((Pz - L1)**2 + (Px_p - a1)**2)

    alpha = sp.atan2((Pz - L1), (Px_p - a1))
    beta = sp.acos((R**2 + a2**2 - a3**2)/(2*a2*R))
    gamma = sp.acos((a2**2 + a3**2 - R**2)/(2*a2*a3))

    theta2 = alpha - beta
    theta3 = sp.pi - gamma

    return (theta1, theta2, theta3), (Px, Py, Pz, L1, a1, a2, a3)


# ------------------ CONTROL CINEMÁTICO ------------------
def control_cinematico():

    # ---- Parámetros ----
    dt = 0.01          # 20 ms entre muestras
    pausa_archivo = 4   # 50 ms entre archivos

    # ---- Ganancia ----
    K = np.array([150, 150, 150]).reshape(3,1)

    # ---- Modelos simbólicos ----
    T_sym, vars_dir = cinematica_directa()
    J_sym, _ = jacobiano()
    (theta1_sym, theta2_sym, theta3_sym), vars_inv = cinematica_inversa()

    # ---- Parámetros robot ----
    valores_robot = [0.19, 0.06, 0.15, 0.17]

    # ---- Serial ESP32 ----
    ser = serial.Serial('COM5', 115200)
    time.sleep(2)

    # =====================================================
    # RECORRER LOS 21 ARCHIVOS
    # =====================================================
    for archivo in range(1, 22):

        print(f"\n========== ARCHIVO {archivo} ==========")

        # ---- Nombres de archivos ----
        archivo_pos = f"Poslineal{archivo}.csv"
        archivo_vel = f"PosLinealpunto{archivo}.csv"

        # ---- Cargar CSV ----
        Xd_array = np.loadtxt(archivo_pos, delimiter=",")
        Xd_punto_array = np.loadtxt(archivo_vel, delimiter=",")

        # ---- Verificación ----
        if Xd_array.shape != Xd_punto_array.shape:
            raise ValueError(f"Error de tamaño en archivo {archivo}")

        num_pasos = Xd_array.shape[0]
        vector_tiempo = np.arange(0, num_pasos*dt, dt)

        # =====================================================
        # CONDICIÓN INICIAL
        # =====================================================
        Pi = Xd_array[0]

        subs_inv = {
            vars_inv[0]: Pi[0],
            vars_inv[1]: Pi[1],
            vars_inv[2]: Pi[2],
            vars_inv[3]: valores_robot[0],
            vars_inv[4]: valores_robot[1],
            vars_inv[5]: valores_robot[2],
            vars_inv[6]: valores_robot[3],
        }

        q = np.array([
            float(theta1_sym.subs(subs_inv)),
            float(theta2_sym.subs(subs_inv)),
            float(theta3_sym.subs(subs_inv))
        ]).reshape(3,1)

        # =====================================================
        # LOOP DE CONTROL
        # =====================================================
        for i, t in enumerate(vector_tiempo):

            Xd = Xd_array[i].reshape(3,1)
            Xd_punto = Xd_punto_array[i].reshape(3,1)

            subs_dir = {
                vars_dir[0]: q[0,0],
                vars_dir[1]: q[1,0],
                vars_dir[2]: q[2,0],
                vars_dir[3]: valores_robot[0],
                vars_dir[4]: valores_robot[1],
                vars_dir[5]: valores_robot[2],
                vars_dir[6]: valores_robot[3],
            }

            # ---- Posición actual ----
            T_num = np.array(T_sym.subs(subs_dir)).astype(float)
            X_actual = T_num[0:3, 3].reshape(3,1)

            # ---- Error ----
            error = Xd - X_actual

            # ---- Control cartesiano ----
            Xe_punto = Xd_punto + K * error

            # ---- Jacobiano ----
            J_num = np.array(J_sym.subs(subs_dir)).astype(float)

            # ---- Velocidad articular ----
            q_punto = np.linalg.pinv(J_num) @ Xe_punto

            # ---- Integración ----
            q = q + q_punto * dt

            # ---- Enviar a ESP32 ----
            data = f"{q[0,0]:.4f},{q[1,0]:.4f},{q[2,0]:.4f}\n"

            ser.write(data.encode())

            print(f"Archivo {archivo} | t={t:.2f} -> {data.strip()}")

            # Espera entre muestras
            time.sleep(dt)

        # =====================================================
        # PAUSA ENTRE ARCHIVOS
        # =====================================================
        print(f"Finalizó archivo {archivo}")
        time.sleep(pausa_archivo)

    # ---- Cerrar serial ----
    ser.close()


# ------------------ MAIN ------------------
if __name__ == "__main__":
    control_cinematico()