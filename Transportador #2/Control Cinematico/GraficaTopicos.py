import sys
import serial
import numpy as np
from PyQt5 import QtWidgets, QtCore
import pyqtgraph as pg

# =============================
# CONFIGURACIÓN SERIAL
# =============================
PORT = 'COM4'     # ⚠️ Cambia esto
BAUD = 115200     # (puedes subirlo a 921600 si cambias en ESP32)

# =============================
# CLASE PRINCIPAL
# =============================
class SerialPlotter(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("Control en Tiempo Real ESP32")
        self.setGeometry(100, 100, 1000, 700)

        # =============================
        # SERIAL (NO BLOQUEANTE)
        # =============================
        self.ser = serial.Serial(PORT, BAUD, timeout=0.001)

        # =============================
        # VARIABLES
        # =============================
        self.N = 500
        self.ref = np.zeros(self.N)
        self.pos = np.zeros(self.N)
        self.ang = np.zeros(self.N)
        self.u   = np.zeros(self.N)
        self.err = np.zeros(self.N)

        # =============================
        # INTERFAZ
        # =============================
        self.widget = pg.GraphicsLayoutWidget()
        self.setCentralWidget(self.widget)

        # ---- POSICIÓN ----
        self.plot1 = self.widget.addPlot(title="Posición")
        self.curve_ref = self.plot1.plot(pen='r', name="Referencia")
        self.curve_pos = self.plot1.plot(pen='g', name="Posición")

        self.widget.nextRow()

        # ---- ÁNGULO ----
        self.plot2 = self.widget.addPlot(title="Ángulo")
        self.curve_ang = self.plot2.plot(pen='y')

        self.widget.nextRow()

        # ---- CONTROL ----
        self.plot3 = self.widget.addPlot(title="Control (u)")
        self.curve_u = self.plot3.plot(pen='c')

        self.widget.nextRow()

        # ---- ERROR ----
        self.plot4 = self.widget.addPlot(title="Error")
        self.curve_err = self.plot4.plot(pen='m')

        # =============================
        # TIMER (más lento = más estable)
        # =============================
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(50)  # 🔥 20 Hz (ANTES 50 Hz)

        # =============================
        # CONTROLES
        # =============================
        self.init_controls()

    # =============================
    # BOTONES
    # =============================
    def init_controls(self):
        toolbar = self.addToolBar("Control")

        btn1 = QtWidgets.QPushButton("Step")
        btn1.clicked.connect(lambda: self.send("1"))
        toolbar.addWidget(btn1)

        btn2 = QtWidgets.QPushButton("Pulsos")
        btn2.clicked.connect(lambda: self.send("2"))
        toolbar.addWidget(btn2)

        btn3 = QtWidgets.QPushButton("Rampas")
        btn3.clicked.connect(lambda: self.send("3"))
        toolbar.addWidget(btn3)

        btn4 = QtWidgets.QPushButton("Seno")
        btn4.clicked.connect(lambda: self.send("4"))
        toolbar.addWidget(btn4)

        self.input_ref = QtWidgets.QLineEdit()
        self.input_ref.setPlaceholderText("Referencia manual")
        toolbar.addWidget(self.input_ref)

        btn_send = QtWidgets.QPushButton("Enviar")
        btn_send.clicked.connect(self.send_manual)
        toolbar.addWidget(btn_send)

    def send(self, msg):
        try:
            self.ser.write((msg + '\n').encode())
        except:
            pass

    def send_manual(self):
        val = self.input_ref.text()
        try:
            self.ser.write((val + '\n').encode())
        except:
            pass

    # =============================
    # ACTUALIZACIÓN (CLAVE)
    # =============================
    def update(self):
        try:
            # 🔥 Leer TODO el buffer disponible
            while self.ser.in_waiting:
                line = self.ser.readline().decode(errors='ignore').strip()
                data = line.split(',')

                if len(data) == 5:
                    try:
                        r, p, a, u, e = map(float, data)

                        # Desplazar señales
                        self.ref = np.roll(self.ref, -1)
                        self.pos = np.roll(self.pos, -1)
                        self.ang = np.roll(self.ang, -1)
                        self.u   = np.roll(self.u, -1)
                        self.err = np.roll(self.err, -1)

                        # Insertar nuevos valores
                        self.ref[-1] = r
                        self.pos[-1] = p
                        self.ang[-1] = a
                        self.u[-1]   = u
                        self.err[-1] = e

                    except:
                        pass

            # 🔥 Actualizar gráficas UNA sola vez
            self.curve_ref.setData(self.ref)
            self.curve_pos.setData(self.pos)
            self.curve_ang.setData(self.ang)
            self.curve_u.setData(self.u)
            self.curve_err.setData(self.err)

        except:
            pass


# =============================
# EJECUCIÓN
# =============================
if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    win = SerialPlotter()
    win.show()
    sys.exit(app.exec_())