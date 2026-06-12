#!/usr/bin/env python3
# Recebe o frame PPM anotado que a ETAPA 4 manda pela serial.
# Uso:
#   1) Feche o `pio device monitor`
#   2) python3 tools/recebe_debug_visao.py /dev/ttyUSB0 115200
#   3) aperte EN na placa ou espere a primeira deteccao
#
# Opcional:
#   python3 tools/recebe_debug_visao.py /dev/ttyUSB0 115200 f
# envia "f" para pedir um novo frame anotado.

import base64
import sys
import time

try:
    import serial
except ImportError:
    print("Falta a lib pyserial. Instale com:  pip install pyserial")
    sys.exit(1)

porta = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB0"
baud = int(sys.argv[2]) if len(sys.argv) > 2 else 115200
# argv[3]: comando a enviar antes de capturar (ex.: f=frame, g=fundo, c=calibra)
comando = sys.argv[3] if len(sys.argv) > 3 else None

INICIO = "===DEBUG_VISAO_BASE64_INICIO==="
FIM = "===DEBUG_VISAO_BASE64_FIM==="
META = "===DEBUG_VISAO_META"

print(f"Abrindo {porta} @ {baud}. Aguardando debug visual da ETAPA 4...")
ser = serial.Serial(porta, baud, timeout=20)
time.sleep(2.0)

if comando:
    print(f"Enviando comando '{comando}'...")
    ser.write((comando + "\n").encode())
    # 'f' sozinho ja pede o frame; outros comandos tambem pedem em seguida
    if comando.lower() != "f":
        time.sleep(0.5)
        ser.write(b"f\n")

capturando = False
linhas = []
ultima_meta = ""

while True:
    raw = ser.readline()
    if not raw:
        print("Timeout esperando debug. A bola foi detectada? Use o argumento final 'f' para pedir um frame.")
        continue

    linha = raw.decode(errors="ignore").strip()
    if linha.startswith(META):
        ultima_meta = linha
        print(linha)
        continue

    if INICIO in linha:
        capturando = True
        linhas = []
        print("Recebendo frame anotado...")
        continue

    if FIM in linha:
        dados = base64.b64decode("".join(linhas))
        with open("debug_bola.ppm", "wb") as f:
            f.write(dados)
        print(f"OK! Salvou debug_bola.ppm ({len(dados)} bytes).")
        try:
            from PIL import Image
            im = Image.open("debug_bola.ppm")
            im.resize((im.width * 3, im.height * 3), Image.NEAREST).save("debug_bola.png")
            print("Tambem salvou debug_bola.png (3x, sem suavizacao).")
        except Exception:
            pass
        if ultima_meta:
            print(f"Meta: {ultima_meta}")
        break

    if capturando:
        linhas.append(linha)
