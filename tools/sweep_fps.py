#!/usr/bin/env python3
# Dispara o SWEEP automatico da ETAPA 4 (varre exposicao e divisor de clock),
# salva um CSV e, se tiver matplotlib, plota fps x cada parametro.
#
# Uso (feche o `pio device monitor` antes):
#   python3 tools/sweep_fps.py /dev/ttyUSB0 115200 sweep_gray.csv
#
# Rode uma vez com o firmware compilado em GRAYSCALE (CAM_CAPTURA_JPEG 0)
# e outra em JPEG (CAM_CAPTURA_JPEG 1), salvando em arquivos diferentes,
# para comparar os dois modos (teste A vs B).

import sys
import time

try:
    import serial
except ImportError:
    print("Falta a lib pyserial. Instale com:  pip install pyserial")
    sys.exit(1)

porta = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB0"
baud = int(sys.argv[2]) if len(sys.argv) > 2 else 115200
saida = sys.argv[3] if len(sys.argv) > 3 else "sweep.csv"

INICIO = "===SWEEP_INICIO"
FIM = "===SWEEP_FIM==="

ser = serial.Serial(porta, baud, timeout=5)
time.sleep(2.0)
print(f"Disparando sweep (envia 'S'). Salvando em {saida} ...")
ser.reset_input_buffer()
ser.write(b"S\n")

modo = "?"
linhas = []
capturando = False
t_limite = time.time() + 180  # seguranca: 3 min

while time.time() < t_limite:
    raw = ser.readline()
    if not raw:
        continue
    linha = raw.decode(errors="ignore").strip()
    if linha.startswith(INICIO):
        capturando = True
        if "modo=" in linha:
            modo = linha.split("modo=")[1].rstrip("=")
        print(f"Sweep iniciado (modo={modo})")
        continue
    if FIM in linha:
        print("Sweep concluido.")
        break
    if capturando and linha.startswith("SWEEP,"):
        # pula a linha de cabecalho (SWEEP,fase,aec,...)
        if linha.startswith("SWEEP,fase"):
            continue
        partes = linha.split(",")[1:]  # tira o prefixo "SWEEP"
        linhas.append(partes)
        print("  " + ",".join(partes))

ser.close()

if not linhas:
    print("Nada capturado. O firmware esta na ETAPA 4? A porta esta certa?")
    sys.exit(1)

with open(saida, "w") as f:
    f.write("modo,fase,aec,agc,clkdiv,fps,cap_ms,proc_ms\n")
    for p in linhas:
        f.write(modo + "," + ",".join(p) + "\n")
print(f"Salvou {saida} ({len(linhas)} pontos).")

# ---- melhor ponto ----
def fps_de(p):
    try:
        return float(p[4])
    except (IndexError, ValueError):
        return 0.0

melhor = max(linhas, key=fps_de)
print(f"Melhor FPS: {fps_de(melhor):.1f}  (fase={melhor[0]} aec={melhor[1]} clkdiv={melhor[3]})")

# ---- plot opcional ----
try:
    import matplotlib.pyplot as plt
except ImportError:
    print("Sem matplotlib (pip install matplotlib) — so salvei o CSV.")
    sys.exit(0)

aec = [(int(p[1]), fps_de(p)) for p in linhas if p[0] == "AEC"]
clk = [(int(p[3]), fps_de(p)) for p in linhas if p[0] == "CLK"]

fig, (a1, a2) = plt.subplots(1, 2, figsize=(11, 4))
if aec:
    a1.plot([x for x, _ in aec], [y for _, y in aec], "o-")
a1.set_title(f"FPS x exposicao (aec) [{modo}]"); a1.set_xlabel("aec"); a1.set_ylabel("fps"); a1.grid(True)
if clk:
    a2.plot([x for x, _ in clk], [y for _, y in clk], "o-", color="tab:red")
a2.set_title(f"FPS x divisor de clock [{modo}]"); a2.set_xlabel("clkdiv (0xD3)"); a2.set_ylabel("fps"); a2.grid(True)
plt.tight_layout()
png = saida.rsplit(".", 1)[0] + ".png"
plt.savefig(png, dpi=110)
print(f"Salvou {png}")
plt.show()
