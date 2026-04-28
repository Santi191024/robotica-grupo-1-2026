import serial
import csv
import time

# --- CONFIGURACIÓN ---
PUERTO = 'COM3'  
BAUDIOS = 115200
ARCHIVO_CSV = 'trayectorias.csv'
# ---------------------

try:
    # Conexión serial
    esp32 = serial.Serial(PUERTO, BAUDIOS, timeout=0.1)
    time.sleep(2) # Espera a que la ESP32 se reinicie tras conectar
    print(f"Conectado a {PUERTO}")

    with open(ARCHIVO_CSV, mode='r') as f:
        lector = csv.reader(f)
        
        for fila in lector:
            if len(fila) < 3:
                continue # Salta líneas vacías o incompletas

            # Formateamos la cadena: "ref1,ref2,ref3\n"
            cadena_envio = f"{fila},{fila},{fila}\n"
            
            # Enviamos a la ESP32
            esp32.write(cadena_envio.encode('utf-8'))
            
            
            respuesta = esp32.readline().decode('utf-8').strip()
            
            print(f"Enviado: {cadena_envio.strip()} | ESP32 reporta: {respuesta}")

            
            time.sleep(0.05)

    print("\nTrayectoria completada exitosamente.")
    esp32.close()

except Exception as e:
    print(f"Error: {e}")