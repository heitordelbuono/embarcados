#!/usr/bin/env python3
# Recebe a telemetria CSV da ETAPA 4 (comando 't' na serial) e plota a
# trajetoria (x,y) e as series temporais. Tambem salva telemetria.csv.
#
# Uso:
#   1) Feche o `pio device monitor`
#   2) python3 tools/plot_telemetria.py /dev/ttyUSB0 115200
#   3) deixe a bola se mover; Ctrl+C para parar e plotar.
#
# Formato das linhas no firmware:
#   CSV,t_us,x,y,vx,vy,achou,fps

import sys
import time

try:
    import serial
except ImportError:
    print("Falta a lib pyserial. Instale com:  pip install pyserial")
    sys.exit(1)

porta = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB0"
baud = int(sys.argv[2]) if len(sys.argv) > 2 else 115200

ser = serial.Serial(porta, baud, timeout=2)
time.sleep(2.0)
print("Ligando telemetria (envia 't')... Ctrl+C para parar e plotar.")
ser.write(b"t\n")

t, x, y, vx, vy, achou = [], [], [], [], [], []
t0 = None

try:
    while True:
        raw = ser.readline()
        if not raw:
            continue
        linha = raw.decode(errors="ignore").strip()
        if not linha.startswith("CSV,"):
            continue
        p = linha.split(",")
        if len(p) < 8:
            continue
        try:
            tus = int(p[1])
        except ValueError:
            continue
        if t0 is None:
            t0 = tus
        t.append((tus - t0) / 1e6)
        x.append(float(p[2])); y.append(float(p[3]))
        vx.append(float(p[4])); vy.append(float(p[5]))
        achou.append(int(p[6]))
        if len(t) % 50 == 0:
            print(f"  {len(t)} amostras...")
except KeyboardInterrupt:
    pass
finally:
    ser.write(b"t\n")  # desliga telemetria
    ser.close()

print(f"Coletadas {len(t)} amostras.")
with open("telemetria.csv", "w") as f:
    f.write("t_s,x_cm,y_cm,vx,vy,achou\n")
    for i in range(len(t)):
        f.write(f"{t[i]:.4f},{x[i]:.3f},{y[i]:.3f},{vx[i]:.3f},{vy[i]:.3f},{achou[i]}\n")
print("Salvou telemetria.csv")

try:
    import matplotlib.pyplot as plt
except ImportError:
    print("Sem matplotlib (pip install matplotlib) — so salvei o CSV.")
    sys.exit(0)

fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(15, 5))
ax1.plot(x, y, ".-", ms=2, lw=0.5)
ax1.set_title("Trajetoria (cm)"); ax1.set_xlabel("x"); ax1.set_ylabel("y")
ax1.set_aspect("equal"); ax1.grid(True); ax1.axhline(0, c="k", lw=0.5); ax1.axvline(0, c="k", lw=0.5)
ax2.plot(t, x, label="x"); ax2.plot(t, y, label="y")
ax2.set_title("Posicao x t"); ax2.set_xlabel("s"); ax2.set_ylabel("cm"); ax2.legend(); ax2.grid(True)
ax3.plot(t, vx, label="vx"); ax3.plot(t, vy, label="vy")
ax3.set_title("Velocidade x t"); ax3.set_xlabel("s"); ax3.set_ylabel("cm/s"); ax3.legend(); ax3.grid(True)
plt.tight_layout()
plt.savefig("telemetria.png", dpi=110)
print("Salvou telemetria.png")
plt.show()
