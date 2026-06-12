#!/usr/bin/env python3
# Recebe a foto JPEG que a ESP32 manda pela serial (modo SEM WiFi).
# Uso:
#   1) FECHE o 'pio device monitor' (so um programa pode usar a porta)
#   2) python3 tools/recebe_foto.py [porta] [baud]
#   3) aperte EN na placa para ela rebootar e mandar a foto
#
# Salva como foto.jpg na pasta atual.

import sys
import base64

try:
    import serial  # pip install pyserial
except ImportError:
    print("Falta a lib pyserial. Instale com:  pip install pyserial")
    sys.exit(1)

porta = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB0"
baud  = int(sys.argv[2]) if len(sys.argv) > 2 else 115200

INICIO = "===FOTO_BASE64_INICIO==="
FIM    = "===FOTO_BASE64_FIM==="

print(f"Abrindo {porta} @ {baud}. Aperte EN na placa para mandar a foto...")
ser = serial.Serial(porta, baud, timeout=20)

capturando = False
linhas = []

while True:
    raw = ser.readline()
    if not raw:
        print("Timeout esperando dados. A placa mandou a foto? Aperte EN.")
        continue
    linha = raw.decode(errors="ignore").strip()

    if INICIO in linha:
        capturando = True
        linhas = []
        print("Recebendo foto...")
        continue
    if FIM in linha:
        dados = base64.b64decode("".join(linhas))
        with open("foto.jpg", "wb") as f:
            f.write(dados)
        print(f"OK! Salvou foto.jpg ({len(dados)} bytes). Abra para conferir.")
        break
    if capturando:
        linhas.append(linha)
