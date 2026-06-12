#!/usr/bin/env python3
# Recebe a foto JPEG que a ESP32 manda pela serial (modo SEM WiFi).
# Uso:
#   1) FECHE o 'pio device monitor' (so um programa pode usar a porta)
#   2) python3 tools/recebe_foto.py [porta] [baud]
#   3) o script tenta resetar a placa pela serial automaticamente
#
# Salva como foto.jpg na pasta atual.

import sys
import base64
import time

try:
    import serial  # pip install pyserial
except ImportError:
    print("Falta a lib pyserial. Instale com:  pip install pyserial")
    sys.exit(1)

porta = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB0"
baud  = int(sys.argv[2]) if len(sys.argv) > 2 else 115200
auto_reset = "--no-reset" not in sys.argv

INICIO = "===FOTO_BASE64_INICIO==="
FIM    = "===FOTO_BASE64_FIM==="

print(f"Abrindo {porta} @ {baud}.")
ser = serial.Serial(porta, baud, timeout=20)

if auto_reset:
    print("Resetando a ESP32 pela serial...")
    ser.dtr = False   # GPIO0 alto: boot normal, nao bootloader
    ser.rts = True    # EN baixo
    time.sleep(0.15)
    ser.rts = False   # EN alto: reinicia a aplicacao
    time.sleep(0.5)
    ser.reset_input_buffer()
else:
    print("Auto-reset desativado. Reinicie a placa manualmente para mandar a foto.")

capturando = False
linhas = []

while True:
    raw = ser.readline()
    if not raw:
        print("Timeout esperando dados. A placa mandou a foto? Tente sem monitor aberto, ou use --no-reset e reinicie manualmente.")
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
