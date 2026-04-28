import sympy as sp
import numpy as np
import serial

# ==============================
# CONFIG SERIAL
# ==============================
ser = serial.Serial('COM4', 115200, timeout=0)  

# ==============================
# FUNCIÓN CINEMÁTICA DIRECTA
# ==============================
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


# ==============================
# JACOBIANO
# ==============================
def jacobiano():
    T, variables = cinematica_directa()
    theta1, theta2, theta3, L1, a1, a2, a3 = variables

    pos = T[0:3, 3]
    Jv = pos.jacobian([theta1, theta2, theta3])

    return Jv, variables


# ==============================
# CINEMÁTICA INVERSA
# ==============================
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


# ==============================
# PLANEACIÓN
# ==============================
def planeacion_trayectoria():
    
    Pix, PiY, PiZ = 0.2955, 0.1731, 0.4323
    PfX, PfY, PfZ = 0.41, 0.11, 0.3411
    vel = 0.3

    D = np.sqrt((PfX-Pix)**2 + (PfY-PiY)**2 + (PfZ-PiZ)**2)

    U = np.array([PfX-Pix, PfY-PiY, PfZ-PiZ])
    vec_vel = vel * U / D

    return Pix, PiY, PiZ, PfX, PfY, PfZ, vec_vel


# ==============================
# TRAYECTORIA + ENVÍO SERIAL
# ==============================
def trayectoria_y_envio(theta1_val, theta2_val, theta3_val, vec_vel, Pix, PiY, PiZ, PfX, PfY, PfZ):

    dt = 0.02  # periodo de muestreo (igual al ESP)
    vel = 0.3

    D = np.sqrt((PfX-Pix)**2 + (PfY-PiY)**2 + (PfZ-PiZ)**2)
    T_total = D / vel

    t_vec = np.arange(0, T_total, dt)

    q = np.array([theta1_val, theta2_val, theta3_val], dtype=float)

    J_sym, variables = jacobiano()
    theta1, theta2, theta3, L1, a1, a2, a3 = variables

    for _ in t_vec:

        valores = {
            theta1: q[0],
            theta2: q[1],
            theta3: q[2],
            L1: 0.27,
            a1: 0.06,
            a2: 0.15,
            a3: 0.29
        }

        J_eval = np.array(sp.N(J_sym.subs(valores)), dtype=float)

        q_dot = np.linalg.inv(J_eval).dot(vec_vel.reshape(3,1))
        q = q + q_dot.flatten() * dt

        # 🔥 CONVERSIÓN A GRADOS
        q_deg = np.degrees(q)

        # 🔥 ENVÍO SERIAL
        mensaje = f"{q_deg[0]},{q_deg[1]},{q_deg[2]}\n"
        ser.write(mensaje.encode())

        # opcional debug
        print(mensaje.strip())


# ==============================
# MAIN
# ==============================
if __name__ == "__main__":

    # Cinemática inversa inicial
    (theta1_s, theta2_s, theta3_s), vars_inv = cinematica_inversa()
    Px, Py, Pz, L1, a1, a2, a3 = vars_inv

    valores_inv = {
        Px: 0.2955,
        Py: 0.1731,
        Pz: 0.4323,
        L1: 0.27,
        a1: 0.06,
        a2: 0.15,
        a3: 0.29
    }

    theta1_val = float(theta1_s.subs(valores_inv))
    theta2_val = float(theta2_s.subs(valores_inv))
    theta3_val = float(theta3_s.subs(valores_inv))

    # Trayectoria
    Pix, PiY, PiZ, PfX, PfY, PfZ, vec_vel = planeacion_trayectoria()

    # Ejecutar + enviar
    trayectoria_y_envio(
        theta1_val, theta2_val, theta3_val,
        vec_vel,
        Pix, PiY, PiZ,
        PfX, PfY, PfZ
    )