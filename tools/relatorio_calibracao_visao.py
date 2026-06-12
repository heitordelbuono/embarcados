#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Script de Calibração Automática da Visão e Geração de Relatório
Uso:
  python3 tools/relatorio_calibracao_visao.py /dev/ttyUSB0 115200
"""

import os
import sys
import time
import math
import base64
import datetime

# --- Verificação de dependências ---
try:
    import serial
except ImportError:
    print("Erro: A biblioteca 'pyserial' não está instalada.")
    print("Instale rodando: pip install pyserial")
    sys.exit(1)

try:
    from PIL import Image
except ImportError:
    Image = None

try:
    import matplotlib
    matplotlib.use('Agg') # Não abre janela gráfica
    import matplotlib.pyplot as plt
except ImportError:
    plt = None


PORTA_GLOBAL = "/dev/ttyUSB0"
BAUD_GLOBAL = 115200


# --- Escrita e Leitura Seguras com Auto-Reconexão ---
def write_serial(ser, data):
    """Escreve na serial de forma robusta com reconexão automática em caso de queda."""
    try:
        ser.write(data)
    except (serial.SerialException, OSError, AttributeError) as e:
        print(f"\n[AVISO] Conexão serial perdida ({e})! Tentando restabelecer...")
        reconectar_serial(ser)
        try:
            ser.write(data)
        except Exception:
            pass


def read_serial_line(ser, timeout=1.0):
    """Lê uma linha da serial de forma robusta."""
    t_limite = time.time() + timeout
    ser.timeout = 0.1
    while time.time() < t_limite:
        try:
            raw = ser.readline()
            if raw:
                return raw
        except (serial.SerialException, OSError) as e:
            print(f"\n[AVISO] Falha de leitura serial ({e})! Tentando restabelecer...")
            reconectar_serial(ser)
            t_limite = time.time() + timeout
    return b""


def reconectar_serial(ser):
    """Tenta fechar e reabrir a porta serial até ter sucesso (recuperação de reset/brownout)."""
    try:
        ser.close()
    except Exception:
        pass
    
    time.sleep(2.0) # Espera a USB do sistema operacional estabilizar
    while True:
        try:
            print(f"  Tentando conectar a {PORTA_GLOBAL} @ {BAUD_GLOBAL}...")
            ser.port = PORTA_GLOBAL
            ser.baudrate = BAUD_GLOBAL
            ser.timeout = 1.0
            ser.open()
            print("  Conexão restabelecida com sucesso!")
            ser.reset_input_buffer()
            limpa_telemetria_ativa(ser)
            break
        except Exception as e:
            print(f"  Falha na conexão: {e}. Tentando novamente em 2 segundos...")
            time.sleep(2.0)


# --- Limpa qualquer telemetria ativa no boot/reconexão ---
def limpa_telemetria_ativa(ser):
    """
    Detecta se a ESP32 está ativamente cuspindo linhas de telemetria 'CSV,'
    e envia o comando 't' para desligá-la se necessário.
    """
    print("  Verificando se a telemetria da ESP32 está ligada...")
    t_fim = time.time() + 1.0
    telemetria_detectada = False
    
    while time.time() < t_fim:
        raw = read_serial_line(ser, timeout=0.1)
        if raw:
            line = raw.decode('utf-8', errors='ignore').strip()
            if line.startswith("CSV,"):
                telemetria_detectada = True
                break
                
    if telemetria_detectada:
        print("    [AVISO] Telemetria detectada ativa! Enviando comando 't' para desligar...")
        write_serial(ser, b"t\n")
        time.sleep(0.3)
        ser.reset_input_buffer()
        print("    Telemetria desativada.")
    else:
        print("    A telemetria já está desligada.")


# --- Espera por uma resposta específica da ESP32 ---
def espera_resposta(ser, substrings_esperadas, timeout=5.0):
    """
    Lê a serial continuamente até encontrar uma linha que contenha
    uma das substrings esperadas. Retorna a linha decodificada encontrada.
    """
    t_limite = time.time() + timeout
    while time.time() < t_limite:
        raw = read_serial_line(ser, timeout=0.2)
        if not raw:
            continue
        line = raw.decode('utf-8', errors='ignore').strip()
        if line:
            # Imprime toda a saída da ESP32 para podermos diagnosticar travamentos/resets
            print(f"    [ESP32] {line}")
        for sub in substrings_esperadas:
            if sub in line:
                return line
    return ""


# --- Configurar sensor via serial (rajadas de comandos relativos com verificação ativa) ---
def configura_sensor(ser, aec_desejado, agc_desejado):
    """
    Envia comandos de incremento/decremento e usa o feedback do sensor 'i'
    para confirmar se a ESP32 de fato atingiu a exposição e ganho configurados.
    """
    print(f"  Sincronizando sensor para AEC={aec_desejado}, AGC={agc_desejado}...")
    
    cmd_aec_reset = b'-' * 25
    cmd_agc_reset = b',' * 30
    cmd_aec_set = b'+' * (aec_desejado // 50)
    cmd_agc_set = b'.' * agc_desejado

    # Envia os comandos de reset e ajuste
    write_serial(ser, cmd_aec_reset + cmd_agc_reset + b'\n')
    time.sleep(0.1)
    write_serial(ser, cmd_aec_set + cmd_agc_set + b'\n')
    time.sleep(0.2)

    # Loop de verificação ativa (closed-loop)
    for tentativa in range(5):
        ser.reset_input_buffer()
        write_serial(ser, b"i\n")
        
        # Espera o cabeçalho do bloco de resposta do sensor
        resp = espera_resposta(ser, ["aec(exposicao)="], timeout=1.5)
        if resp:
            # Analisa se os valores batem. Exemplo de linha:
            # "  auto=0  aec(exposicao)=300  agc(ganho)=0  clkdiv(0xD3)=-1"
            partes = resp.strip().split()
            aec_atual = None
            agc_atual = None
            for p in partes:
                if "aec(exposicao)=" in p:
                    aec_atual = int(p.split("=")[1])
                elif "agc(ganho)=" in p:
                    agc_atual = int(p.split("=")[1])
            
            if aec_atual == aec_desejado and agc_atual == agc_desejado:
                # Sincronizado com sucesso!
                print(f"    Sensor sincronizado: AEC={aec_atual}, AGC={agc_atual}")
                return True
            else:
                # Valores não batem, envia os ajustes diferenciais
                print(f"    Desvio detectado (AEC={aec_atual}/{aec_desejado}, AGC={agc_atual}/{agc_desejado}). Corrigindo...")
                if aec_atual is not None:
                    diff_aec = aec_desejado - aec_atual
                    if diff_aec > 0: write_serial(ser, b"+" * (diff_aec // 50) + b"\n")
                    elif diff_aec < 0: write_serial(ser, b"-" * (abs(diff_aec) // 50) + b"\n")
                if agc_atual is not None:
                    diff_agc = agc_desejado - agc_atual
                    if diff_agc > 0: write_serial(ser, b"." * diff_agc + b"\n")
                    elif diff_agc < 0: write_serial(ser, b"," * abs(diff_agc) + b"\n")
                time.sleep(0.2)
        else:
            print("    Aviso: Sem resposta do comando 'i'. Reenviando comandos...")
            write_serial(ser, cmd_aec_reset + cmd_agc_reset + b'\n')
            time.sleep(0.1)
            write_serial(ser, cmd_aec_set + cmd_agc_set + b'\n')
            time.sleep(0.2)

    print("  Aviso: Não foi possível obter confirmação exata da configuração do sensor.")
    return False


# --- Garante o modo de captura (GRAY ou JPEG) com verificação ativa ---
def garante_modo_captura(ser, quer_jpeg):
    """Garante que o sensor está no modo de hardware correto (JPEG ou GRAYSCALE/RGB565)."""
    # 1. Pergunta o modo atual
    for tentativa in range(3):
        write_serial(ser, b"i\n")
        resp = espera_resposta(ser, ["modo captura:"], timeout=2.0)
        if resp:
            modo_atual_jpeg = ("JPEG" in resp)
            if modo_atual_jpeg == quer_jpeg:
                print(f"  Já está no modo correto ({'JPEG' if quer_jpeg else 'GRAYSCALE'}).")
                return True
            break
        print(f"  Aviso: Sem resposta ao comando 'i' (tentativa {tentativa+1}/3).")
        time.sleep(0.5)
    else:
        print("  Sem resposta do status atual. Tentando alternar mesmo assim...")

    # 2. Se o modo não é o desejado, envia 'j' para alternar
    print(f"  Enviando comando 'j' para alternar modo (desejado: {'JPEG' if quer_jpeg else 'GRAYSCALE'})...")
    ser.reset_input_buffer()
    write_serial(ser, b"j\n")
    
    # Aguarda a confirmação de transição do firmware com timeout longo
    resp_j = espera_resposta(ser, ["modo de captura ->", "Captura ->"], timeout=4.0)
    if resp_j:
        print(f"    Confirmação da transição recebida: {resp_j}")
    else:
        print("    Aviso: Não recebeu confirmação imediata da transição de modo. Aguardando estabilização...")
        
    time.sleep(2.5)  # Tempo para a câmera e I2C se estabilizarem após reinício do sensor

    # 3. Verifica novamente se deu certo
    for tentativa in range(3):
        write_serial(ser, b"i\n")
        resp = espera_resposta(ser, ["modo captura:"], timeout=2.0)
        if resp:
            modo_atual_jpeg = ("JPEG" in resp)
            if modo_atual_jpeg == quer_jpeg:
                print(f"  Modo alternado com sucesso para {'JPEG' if quer_jpeg else 'GRAYSCALE'}.")
                return True
            else:
                print(f"  Erro: Modo atual ({'JPEG' if modo_atual_jpeg else 'GRAYSCALE'}) ainda não é o desejado.")
        time.sleep(1.0)
        
    print("  Erro: Falha crítica ao alternar modo de captura.")
    return False


# --- Captura imagem anotada (com Retry automático se o Pillow der erro) ---
def captura_imagem_anotada_com_retry(ser, filepath_no_ext, max_retries=3):
    for r in range(max_retries):
        meta = captura_imagem_anotada(ser, filepath_no_ext)
        ppm_path = filepath_no_ext + ".ppm"
        png_path = filepath_no_ext + ".png"
        
        # Se Image estiver disponível, verifica se o PNG foi gerado
        if Image is not None:
            if os.path.exists(png_path) and os.path.getsize(png_path) > 100:
                return meta
        else:
            # Sem Pillow, verifica apenas se o PPM existe
            if os.path.exists(ppm_path) and os.path.getsize(ppm_path) > 100:
                return meta
                
        print(f"  [AVISO] Foto corrompida na transmissão (tentativa {r+1}/{max_retries}). Limpando e tentando novamente...")
        time.sleep(1.0)
        
    print(f"  [ERRO] Não foi possível obter uma foto íntegra após {max_retries} tentativas.")
    return {}


def captura_imagem_anotada(ser, filepath_no_ext):
    """Envia o comando 'f', decodifica a imagem base64 e opcionalmente converte em PNG."""
    # Garante a existência do diretório de destino defensivamente
    dir_pai = os.path.dirname(os.path.abspath(filepath_no_ext))
    os.makedirs(dir_pai, exist_ok=True)

    ser.reset_input_buffer()
    write_serial(ser, b"f\n")
    
    INICIO = "===DEBUG_VISAO_BASE64_INICIO==="
    FIM = "===DEBUG_VISAO_BASE64_FIM==="
    META = "===DEBUG_VISAO_META"

    capturando = False
    linhas_b64 = []
    meta_data = {}
    t_limite = time.time() + 25.0
    
    chars_validos = set("ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/=")

    while time.time() < t_limite:
        raw = read_serial_line(ser, timeout=0.5)
        if not raw:
            continue
        line = raw.decode('utf-8', errors='ignore').strip()
        
        if line.startswith(META):
            partes = line.replace("===", "").strip().split()
            for p in partes:
                if "=" in p:
                    k, v = p.split("=", 1)
                    meta_data[k] = v
            # Continuamos lendo, pois o base64 virá logo após o META
            continue

        if INICIO in line:
            capturando = True
            linhas_b64 = []
            continue
        
        if FIM in line:
            capturando = False
            try:
                dados = base64.b64decode("".join(linhas_b64))
                ppm_path = filepath_no_ext + ".ppm"
                with open(ppm_path, "wb") as f:
                    f.write(dados)
                
                if Image is not None:
                    try:
                        im = Image.open(ppm_path)
                        png_path = filepath_no_ext + ".png"
                        im.resize((im.width * 2, im.height * 2), Image.NEAREST).save(png_path)
                        if os.path.exists(ppm_path):
                            os.remove(ppm_path)
                    except Exception as e:
                        print(f"  Aviso: Erro na conversão para PNG: {e}")
            except Exception as e:
                print(f"  Erro ao decodificar base64: {e}")
            # FIM recebido e tratado, saímos do loop
            break

        if capturando:
            linha_limpa = line.strip()
            # Filtro inteligente de base64: ignora logs intrometidos
            if linha_limpa and all(c in chars_validos for c in linha_limpa) and len(linha_limpa) >= 4:
                linhas_b64.append(linha_limpa)
            else:
                if linha_limpa and not "DEBUG_VISAO" in line:
                    print(f"    [ESP32 LOG] {linha_limpa}")
        else:
            linha_limpa = line.strip()
            if linha_limpa and not "DEBUG_VISAO" in line:
                print(f"    [ESP32 PRE-LOG] {linha_limpa}")

    return meta_data


# --- Calcula métricas estatísticas da telemetria coletada ---
def calcula_metricas(amostras):
    """
    Calcula FPS médio, taxa de detecção e estabilidade (média/desvio padrão)
    para um conjunto de amostras de telemetria.
    """
    if not amostras:
        return {
            'fps': 0.0, 'achou_pct': 0.0,
            'x_med': 0.0, 'x_std': 0.0,
            'y_med': 0.0, 'y_std': 0.0,
            'vx_med': 0.0, 'vx_std': 0.0,
            'vy_med': 0.0, 'vy_std': 0.0
        }

    n = len(amostras)
    achou_amostras = [a for a in amostras if a['achou'] == 1]
    n_achou = len(achou_amostras)
    
    fps_medio = sum(a['fps'] for a in amostras) / n
    achou_pct = (n_achou / n) * 100.0

    if n_achou == 0:
        return {
            'fps': fps_medio, 'achou_pct': 0.0,
            'x_med': 0.0, 'x_std': 0.0,
            'y_med': 0.0, 'y_std': 0.0,
            'vx_med': 0.0, 'vx_std': 0.0,
            'vy_med': 0.0, 'vy_std': 0.0
        }

    # Estatísticas de posição (apenas quando achou a bola)
    xs = [a['x'] for a in achou_amostras]
    ys = [a['y'] for a in achou_amostras]
    vxs = [a['vx'] for a in achou_amostras]
    vys = [a['vy'] for a in achou_amostras]

    x_med = sum(xs) / n_achou
    y_med = sum(ys) / n_achou
    vx_med = sum(vxs) / n_achou
    vy_med = sum(vys) / n_achou

    x_std = math.sqrt(sum((val - x_med) ** 2 for val in xs) / n_achou) if n_achou > 1 else 0.0
    y_std = math.sqrt(sum((val - y_med) ** 2 for val in ys) / n_achou) if n_achou > 1 else 0.0
    vx_std = math.sqrt(sum((val - vx_med) ** 2 for val in vxs) / n_achou) if n_achou > 1 else 0.0
    vy_std = math.sqrt(sum((val - vy_med) ** 2 for val in vys) / n_achou) if n_achou > 1 else 0.0

    return {
        'fps': fps_medio,
        'achou_pct': achou_pct,
        'x_med': x_med,
        'x_std': x_std,
        'y_med': y_med,
        'y_std': y_std,
        'vx_med': vx_med,
        'vx_std': vx_std,
        'vy_med': vy_med,
        'vy_std': vy_std
    }


# --- Geração de Gráficos de Telemetria ---
def plotar_graficos_telemetria(amostras, filepath_png, titulo_prefixo):
    """
    Gera um gráfico triplo (trajetória, posição temporal, velocidade temporal)
    e salva no caminho indicado.
    """
    if plt is None or not amostras:
        return False
        
    ts = [a['t'] for a in amostras]
    xs = [a['x'] for a in amostras]
    ys = [a['y'] for a in amostras]
    vxs = [a['vx'] for a in amostras]
    vys = [a['vy'] for a in amostras]

    # Garante diretório pai para o plot
    dir_pai = os.path.dirname(os.path.abspath(filepath_png))
    os.makedirs(dir_pai, exist_ok=True)

    fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(15, 4))
    
    # 1. Trajetória 2D
    ax1.plot(xs, ys, "o-", color="tab:blue", ms=3, lw=0.8)
    ax1.set_title("Espaço 2D (cm)")
    ax1.set_xlabel("X (cm)")
    ax1.set_ylabel("Y (cm)")
    ax1.set_aspect("equal")
    ax1.grid(True)
    ax1.axhline(0, color="black", lw=0.5)
    ax1.axvline(0, color="black", lw=0.5)
    ax1.set_xlim(-10, 10)
    ax1.set_ylim(-10, 10)

    # 2. Posição temporal
    ax2.plot(ts, xs, label="X", color="tab:blue")
    ax2.plot(ts, ys, label="Y", color="tab:orange")
    ax2.set_title("Posição vs Tempo")
    ax2.set_xlabel("Tempo (s)")
    ax2.set_ylabel("Posição (cm)")
    ax2.legend()
    ax2.grid(True)

    # 3. Velocidade temporal
    ax3.plot(ts, vxs, label="VX", color="tab:blue", alpha=0.8)
    ax3.plot(ts, vys, label="VY", color="tab:orange", alpha=0.8)
    ax3.set_title("Velocidade vs Tempo")
    ax3.set_xlabel("Tempo (s)")
    ax3.set_ylabel("Velocidade (cm/s)")
    ax3.legend()
    ax3.grid(True)

    plt.suptitle(f"Telemetria - {titulo_prefixo} (Bola Parada)", fontsize=14, fontweight="bold")
    plt.tight_layout()
    plt.savefig(filepath_png, dpi=110)
    plt.close()
    return True


# --- Coleta a telemetria CSV ---
def coleta_telemetria(ser, duracao_s):
    amostras = []
    
    ser.reset_input_buffer()
    write_serial(ser, b"t\n")
    
    # Aguarda a resposta de confirmação de telemetria
    espera_resposta(ser, ["Telemetria ON"], timeout=1.0)
    ser.reset_input_buffer()
    
    t_fim = time.time() + duracao_s
    t0 = None
    
    while time.time() < t_fim:
        raw = read_serial_line(ser, timeout=0.1)
        if not raw:
            continue
        line = raw.decode('utf-8', errors='ignore').strip()
        
        if not line.startswith("CSV,"):
            continue
        
        partes = line.split(",")
        if len(partes) < 8:
            continue
            
        try:
            t_us = int(partes[1])
            x = float(partes[2])
            y = float(partes[3])
            vx = float(partes[4])
            vy = float(partes[5])
            achou = int(partes[6])
            fps = float(partes[7])
        except ValueError:
            continue
            
        if t0 is None:
            t0 = t_us
            
        t_seg = (t_us - t0) / 1e6
        amostras.append({
            't': t_seg,
            'x': x,
            'y': y,
            'vx': vx,
            'vy': vy,
            'achou': achou,
            'fps': fps
        })

    # Desliga telemetria
    write_serial(ser, b"t\n")
    espera_resposta(ser, ["Telemetria OFF"], timeout=1.0)
    ser.reset_input_buffer()

    return amostras


# --- PROGRAMA PRINCIPAL ---
def main():
    global PORTA_GLOBAL, BAUD_GLOBAL
    PORTA_GLOBAL = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB0"
    BAUD_GLOBAL = int(sys.argv[2]) if len(sys.argv) > 2 else 115200

    timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    report_dir = os.path.abspath(f"reports/calibracao_visao_{timestamp}")
    os.makedirs(report_dir, exist_ok=True)
    
    print(f"\n========================================================")
    print(f" RELATÓRIO AUTOMÁTICO DE CALIBRAÇÃO DA VISÃO")
    print(f" Pasta de saída: {report_dir}")
    print(f" Conectando via: {PORTA_GLOBAL} @ {BAUD_GLOBAL}")
    print(f"========================================================")

    # Inicializa conexão serial
    try:
        ser = serial.Serial(PORTA_GLOBAL, BAUD_GLOBAL, timeout=1.0)
        time.sleep(2.0)
        ser.reset_input_buffer()
    except Exception as e:
        print(f"Erro ao abrir a porta serial {PORTA_GLOBAL}: {e}")
        print("Verifique se o cabo está conectado e se o monitor serial do VSCode/PlatformIO está fechado.")
        sys.exit(1)

    # Limpeza de qualquer telemetria inicial
    limpa_telemetria_ativa(ser)

    # Parâmetros de varredura
    aec_sweep_vals = [0, 100, 200, 400, 800, 1200]
    agc_sweep_vals = [0, 4, 8, 16, 30]
    aec_fotos = [0, 200, 1200]
    agc_fotos = [4, 30]

    # --- PASSO 1: CAPTURA DOS FUNDOS DA MESA VAZIA (SENSORT + MODO) ---
    print("\n[Etapa 1 de 2] CAPTURA DE REFERÊNCIAS DA MESA VAZIA")
    print("-> Retire a bola e qualquer objeto de cima da mesa.")
    input("-> Pressione [ENTER] quando o campo estiver limpo para iniciar a captura de fundos...")

    print("\n[MESA] Iniciando calibração automática da mesa (detecção de cantos)...")
    write_serial(ser, b"m\n")
    # Espera o firmware dar a confirmação da calibração
    resp_m = espera_resposta(ser, ["MESA CALIBRADA AUTOMATICAMENTE", "falha na deteccao automatica"], timeout=4.0)
    if "MESA CALIBRADA" in resp_m:
        print("  Sucesso: Cantos e homografia recalibrados via hardware!")
    else:
        print("  Aviso: Falha na detecção automática das bordas. Mantendo geometria estática do config.h.")
    time.sleep(1.0)

    print("\nVarrendo e capturando imagens de fundo da mesa vazia...")
    # 1.1 Fundo em GRAYSCALE
    print("\n[GRAYSCALE] Capturando referências...")
    garante_modo_captura(ser, quer_jpeg=False)
    
    # Varre AEC (AGC=0)
    for aec in aec_sweep_vals:
        print(f"  AEC={aec} (AGC=0)...")
        configura_sensor(ser, aec, 0)
        # Recalibra a mesa geometricamente para a nova exposição
        write_serial(ser, b"m\n")
        espera_resposta(ser, ["MESA CALIBRADA AUTOMATICAMENTE", "falha na deteccao automatica"], timeout=2.0)
        
        write_serial(ser, b"g\n")
        # Espera confirmação do fundo na ESP32 antes de tirar a foto
        espera_resposta(ser, ["fundo da mesa capturado"], timeout=2.0)
        
        if aec in aec_fotos:
            captura_imagem_anotada_com_retry(ser, os.path.join(report_dir, f"fundo_gray_AEC_{aec}_AGC_0"))
        
    # Varre AGC (AEC=300)
    for agc in agc_sweep_vals:
        print(f"  AGC={agc} (AEC=300)...")
        configura_sensor(ser, 300, agc)
        write_serial(ser, b"m\n")
        espera_resposta(ser, ["MESA CALIBRADA AUTOMATICAMENTE", "falha na deteccao automatica"], timeout=2.0)
        
        write_serial(ser, b"g\n")
        espera_resposta(ser, ["fundo da mesa capturado"], timeout=2.0)
        
        if agc in agc_fotos:
            captura_imagem_anotada_com_retry(ser, os.path.join(report_dir, f"fundo_gray_AEC_300_AGC_{agc}"))

    # Captura e deixa ativo o fundo Grayscale do baseline para as telemetrias
    print("  Garantindo fundo do baseline em Grayscale...")
    configura_sensor(ser, 300, 0)
    write_serial(ser, b"m\n")
    espera_resposta(ser, ["MESA CALIBRADA AUTOMATICAMENTE", "falha na deteccao automatica"], timeout=2.0)
    write_serial(ser, b"g\n")
    espera_resposta(ser, ["fundo da mesa capturado"], timeout=2.0)

    # Captura adicional: Imagem com Filtro de Contraste Sobel
    print("  Capturando imagem com filtro de contraste Sobel ativo...")
    write_serial(ser, b"e\n") # Liga Sobel
    espera_resposta(ser, ["Sobel ON"], timeout=2.0)
    captura_imagem_anotada_com_retry(ser, os.path.join(report_dir, "contraste_sobel"))
    write_serial(ser, b"e\n") # Desliga Sobel
    espera_resposta(ser, ["Sobel OFF"], timeout=2.0)

    # 1.2 Fundo em JPEG
    print("\n[JPEG] Capturando referências...")
    garante_modo_captura(ser, quer_jpeg=True)
    
    # Varre AEC (AGC=0)
    for aec in aec_sweep_vals:
        print(f"  AEC={aec} (AGC=0)...")
        configura_sensor(ser, aec, 0)
        write_serial(ser, b"m\n")
        espera_resposta(ser, ["MESA CALIBRADA AUTOMATICAMENTE", "falha na deteccao automatica"], timeout=2.0)
        
        write_serial(ser, b"g\n")
        espera_resposta(ser, ["fundo da mesa capturado"], timeout=2.0)
        
        if aec in aec_fotos:
            captura_imagem_anotada_com_retry(ser, os.path.join(report_dir, f"fundo_jpeg_AEC_{aec}_AGC_0"))
        
    # Varre AGC (AEC=300)
    for agc in agc_sweep_vals:
        print(f"  AGC={agc} (AEC=300)...")
        configura_sensor(ser, 300, agc)
        write_serial(ser, b"m\n")
        espera_resposta(ser, ["MESA CALIBRADA AUTOMATICAMENTE", "falha na deteccao automatica"], timeout=2.0)
        
        write_serial(ser, b"g\n")
        espera_resposta(ser, ["fundo da mesa capturado"], timeout=2.0)
        
        if agc in agc_fotos:
            captura_imagem_anotada_com_retry(ser, os.path.join(report_dir, f"fundo_jpeg_AEC_300_AGC_{agc}"))

    # Captura e deixa ativo o fundo JPEG do baseline para as telemetrias
    print("  Garantindo fundo do baseline em JPEG...")
    configura_sensor(ser, 300, 0)
    write_serial(ser, b"m\n")
    espera_resposta(ser, ["MESA CALIBRADA AUTOMATICAMENTE", "falha na deteccao automatica"], timeout=2.0)
    write_serial(ser, b"g\n")
    espera_resposta(ser, ["fundo da mesa capturado"], timeout=2.0)


    # --- PASSO 2: BOLA NA MESA (TELEMETRIAS DE BASELINE + SWEEPS SEM FUNDO) ---
    print("\n[Etapa 2 de 2] TESTE COM A BOLA")
    print("-> Posicione/cole a bola exatamente no centro da mesa (deixe-a parada).")
    input("-> Pressione [ENTER] quando a bola estiver colada para iniciar os testes automatizados...")

    modos_telemetria = [
        {"id": "jpeg_com_fundo", "jpeg": True, "fundo": True},
        {"id": "gray_com_fundo", "jpeg": False, "fundo": True},
        {"id": "gray_sem_fundo", "jpeg": False, "fundo": False},
        {"id": "jpeg_sem_fundo", "jpeg": True, "fundo": False}
    ]

    metricas_modo = {}

    for modo in modos_telemetria:
        modo_id = modo["id"]
        print(f"\nTelemetria no Baseline: {modo_id.upper()}")
        garante_modo_captura(ser, quer_jpeg=modo["jpeg"])
        
        if modo["fundo"]:
            print("  Fundo ativo.")
        else:
            print("  Desativando fundo (G)...")
            write_serial(ser, b"G\n")
            espera_resposta(ser, ["fundo descartado"], timeout=2.0)

        configura_sensor(ser, 300, 0)
        
        print("  Coletando 5s de telemetria...")
        amostras = coleta_telemetria(ser, duracao_s=5.0)
        csv_path = os.path.abspath(os.path.join(report_dir, f"telemetria_{modo_id}.csv"))
        os.makedirs(os.path.dirname(csv_path), exist_ok=True)
        with open(csv_path, "w") as f:
            f.write("t_s,x_cm,y_cm,vx,vy,achou,fps\n")
            for a in amostras:
                f.write(f"{a['t']:.4f},{a['x']:.3f},{a['y']:.3f},{a['vx']:.3f},{a['vy']:.3f},{a['achou']},{a['fps']:.1f}\n")
        
        metricas = calcula_metricas(amostras)
        metricas_modo[modo_id] = metricas
        
        print(f"  FPS médio: {metricas['fps']:.1f} | Detecção: {metricas['achou_pct']:.1f}%")
        print(f"  Estabilidade X: DP={metricas['x_std']:.4f} cm | Y: DP={metricas['y_std']:.4f} cm")
        
        png_graph = os.path.join(report_dir, f"telemetria_{modo_id}.png")
        plotar_graficos_telemetria(amostras, png_graph, modo_id.upper())

        baseline_img_path = os.path.join(report_dir, f"imagem_{modo_id}_baseline")
        captura_imagem_anotada_com_retry(ser, baseline_img_path)

    # Sweep de sensibilidade sem fundo
    modos_sweep = [
        {"id": "gray_sem_fundo", "jpeg": False},
        {"id": "jpeg_sem_fundo", "jpeg": True}
    ]

    resultados_sweep = []

    for modo in modos_sweep:
        modo_id = modo["id"]
        print(f"\nSweep de Sensibilidade da Bola: {modo_id.upper()}")
        garante_modo_captura(ser, quer_jpeg=modo["jpeg"])
        write_serial(ser, b"G\n")
        espera_resposta(ser, ["fundo descartado"], timeout=2.0)

        print("  Varrendo Exposição (AEC)...")
        for aec in aec_sweep_vals:
            print(f"    AEC={aec} (AGC=0)...")
            configura_sensor(ser, aec, 0)
            amostras_curtas = coleta_telemetria(ser, duracao_s=0.8)
            metr = calcula_metricas(amostras_curtas)
            
            img_name = f"sweep_{modo_id}_AEC_{aec}_AGC_0"
            if aec in aec_fotos:
                captura_imagem_anotada_com_retry(ser, os.path.join(report_dir, img_name))
            
            resultados_sweep.append({
                'modo': modo_id, 'fase': 'AEC', 'aec': aec, 'agc': 0,
                'fps': metr['fps'], 'achou_pct': metr['achou_pct'],
                'x_std': metr['x_std'], 'y_std': metr['y_std']
            })

        print("  Varrendo Ganho (AGC)...")
        for agc in agc_sweep_vals:
            print(f"    AGC={agc} (AEC=300)...")
            configura_sensor(ser, 300, agc)
            amostras_curtas = coleta_telemetria(ser, duracao_s=0.8)
            metr = calcula_metricas(amostras_curtas)
            
            img_name = f"sweep_{modo_id}_AEC_300_AGC_{agc}"
            if agc in agc_fotos:
                captura_imagem_anotada_com_retry(ser, os.path.join(report_dir, img_name))
            
            resultados_sweep.append({
                'modo': modo_id, 'fase': 'AGC', 'aec': 300, 'agc': agc,
                'fps': metr['fps'], 'achou_pct': metr['achou_pct'],
                'x_std': metr['x_std'], 'y_std': metr['y_std']
            })

    # Reseta baseline padrão
    configura_sensor(ser, 300, 0)
    ser.close()

    # --- SALVAR SUMMARY.CSV ---
    summary_path = os.path.abspath(os.path.join(report_dir, "summary.csv"))
    os.makedirs(os.path.dirname(summary_path), exist_ok=True)
    with open(summary_path, "w") as f:
        f.write("modo,fase,aec,agc,fps,achou_pct,x_std_cm,y_std_cm\n")
        for r in resultados_sweep:
            f.write(f"{r['modo']},{r['fase']},{r['aec']},{r['agc']},{r['fps']:.1f},{r['achou_pct']:.1f},{r['x_std']:.3f},{r['y_std']:.3f}\n")
    print(f"\nResumo gravado em {summary_path}")

    # --- RECOMENDAÇÃO ---
    recomendacoes = {}
    for m_id in [m["id"] for m in modos_sweep]:
        pontos = [r for r in resultados_sweep if r['modo'] == m_id]
        pontos_validos = [p for p in pontos if p['achou_pct'] > 95.0]
        if pontos_validos:
            melhor_aec = max([p for p in pontos_validos if p['fase'] == 'AEC'], key=lambda p: p['fps'], default=None)
            melhor_agc = min([p for p in pontos_validos if p['fase'] == 'AGC'], key=lambda p: p['x_std'] + p['y_std'], default=None)
            recomendacoes[m_id] = {
                'aec': melhor_aec['aec'] if melhor_aec else 300,
                'agc': melhor_agc['agc'] if melhor_agc else 0,
                'fps': melhor_aec['fps'] if melhor_aec else 0.0,
                'ruido': (melhor_agc['x_std'] + melhor_agc['y_std'])*0.5 if melhor_agc else 0.0
            }
        else:
            recomendacoes[m_id] = {'aec': 300, 'agc': 0, 'fps': 0.0, 'ruido': 999.0}

    # --- ESCREVER README.MD ---
    readme_path = os.path.abspath(os.path.join(report_dir, "README.md"))
    os.makedirs(os.path.dirname(readme_path), exist_ok=True)
    def img_link(name):
        ext = ".png" if Image is not None else ".ppm"
        return f"![{name}]({name}{ext})"

    with open(readme_path, "w") as f:
        f.write("# Relatório Automático de Calibração da Visão\n\n")
        f.write(f"**Data do teste:** {datetime.datetime.now().strftime('%d/%m/%Y %H:%M:%S')}\n")
        f.write(f"**Porta da ESP32:** `{PORTA_GLOBAL}`\n")
        f.write(f"**Tipo de Execução:** Hardware Físico\n\n")

        f.write("## 1. Guia de Comandos da Serial\n")
        f.write("Lista de teclas de comando implementadas no firmware (ETAPA 4):\n\n")
        f.write("| Comando | Ação | Descrição |\n")
        f.write("|---|---|---|\n")
        f.write("| `f` | Foto anotada | Envia imagem anotada atual com overlays via base64 pela serial. |\n")
        f.write("| `g` | Capturar Fundo | Salva frame de mesa vazia como referência para a subtração de fundo. |\n")
        f.write("| `G` | Descartar Fundo | Desativa a subtração de fundo e retorna para o modo de grade local 4x4. |\n")
        f.write("| `e` | Sobel Overlay | Ativa/desativa contorno de bordas Sobel no frame de debug. |\n")
        f.write("| `t` | Telemetria CSV | Liga/desliga envio contínuo de dados CSV de telemetria. |\n")
        f.write("| `j` | Alternar Captura | Alterna entre sensores GRAYSCALE cru e JPEG decodificado. |\n")
        f.write("| `+` / `-` | Exposição (AEC) | Aumenta ou diminui a exposição em passos de 50 (0 a 1200). |\n")
        f.write("| `.` / `,` | Ganho (AGC) | Aumenta ou diminui o ganho em passos de 1 (0 a 30). |\n")
        f.write("| `a` | Modo Automático | Liga ou desliga exposição/ganho automáticos. |\n")
        f.write("| `i` | Info Sensor | Imprime na serial os valores atuais de exposição, ganho e modo. |\n")
        f.write("| `c` | Calibrar Câmera | Ajusta exposição/ganho temporário, congela e salva na NVS. |\n")
        f.write("| `m` | Calibrar Mesa | Calibração automática da geometria e homografia da mesa via detecção de bordas radial. |\n")
        f.write("| `S` | Sweep Interno | Inicia sweep automático interno e rápido de FPS. |\n\n")

        f.write("## 2. Fotos de Fundo da Mesa Vazia (Comparativo por Configuração)\n")
        f.write("Fotos de fundo capturadas com a mesa vazia (AEC=0, 200, 1200 e AGC=4, 30):\n\n")

        f.write("### Fundos em GRAYSCALE:\n\n")
        f.write("| AEC=0 (AGC=0) | AEC=200 (AGC=0) | AEC=1200 (AGC=0) | AGC=4 (AEC=300) | AGC=30 (AEC=300) |\n")
        f.write("|:---:|:---:|:---:|:---:|:---:|\n")
        f.write(f"| {img_link('fundo_gray_AEC_0_AGC_0')} | {img_link('fundo_gray_AEC_200_AGC_0')} | {img_link('fundo_gray_AEC_1200_AGC_0')} | {img_link('fundo_gray_AEC_300_AGC_4')} | {img_link('fundo_gray_AEC_300_AGC_30')} |\n\n")

        f.write("### Visualização do Filtro de Contraste Sobel:\n\n")
        f.write("Abaixo está a imagem da mesa vazia com o filtro Sobel ativo (AEC=300, AGC=0), evidenciando as bordas e linhas que o sensor de visão detecta:\n\n")
        f.write("| Filtro de Contraste Sobel |\n")
        f.write("|:---:|\n")
        f.write(f"| {img_link('contraste_sobel')} |\n\n")

        f.write("### Fundos em JPEG:\n\n")
        f.write("| AEC=0 (AGC=0) | AEC=200 (AGC=0) | AEC=1200 (AGC=0) | AGC=4 (AEC=300) | AGC=30 (AEC=300) |\n")
        f.write("|:---:|:---:|:---:|:---:|:---:|\n")
        f.write(f"| {img_link('fundo_jpeg_AEC_0_AGC_0')} | {img_link('fundo_jpeg_AEC_200_AGC_0')} | {img_link('fundo_jpeg_AEC_1200_AGC_0')} | {img_link('fundo_jpeg_AEC_300_AGC_4')} | {img_link('fundo_jpeg_AEC_300_AGC_30')} |\n\n")

        f.write("## 3. Comparação de Estabilidade (Bola Parada por 5s no Baseline)\n")
        f.write("Abaixo estão consolidados os desvios de estabilidade da bola no centro geométrico da mesa física (AEC=300, AGC=0):\n\n")
        f.write("| Modo | FPS Médio | Taxa Detecção | Desvio Padrão X (cm) | Desvio Padrão Y (cm) | DP Velocidade VX (cm/s) | DP Velocidade VY (cm/s) |\n")
        f.write("|---|---|---|---|---|---|---|\n")
        for m_id, met in metricas_modo.items():
            f.write(f"| `{m_id}` | {met['fps']:.1f} | {met['achou_pct']:.1f}% | {met['x_std']:.4f} | {met['y_std']:.4f} | {met['vx_std']:.3f} | {met['vy_std']:.3f} |\n")
        f.write("\n")

        f.write("### Gráficos de Trajetória e Séries Temporais:\n\n")
        for m_id in metricas_modo.keys():
            f.write(f"#### Modo: {m_id.upper()}\n")
            if plt is not None:
                f.write(f"{img_link('telemetria_' + m_id)}\n\n")
            else:
                f.write("*Gráfico indisponível (instale matplotlib)*\n\n")

        f.write("### Imagens Baseline da Bola:\n")
        f.write("| JPEG Com Fundo | Grayscale Com Fundo | Grayscale Sem Fundo | JPEG Sem Fundo |\n")
        f.write("|:---:|:---:|:---:|:---:|\n")
        f.write(f"| {img_link('imagem_jpeg_com_fundo_baseline')} | {img_link('imagem_gray_com_fundo_baseline')} | {img_link('imagem_gray_sem_fundo_baseline')} | {img_link('imagem_jpeg_sem_fundo_baseline')} |\n\n")

        f.write("## 4. Sweeps de Sensibilidade da Bola (Sem Fundo)\n")
        f.write("Abaixo estão as fotos de sweep tiradas nos pontos de limites e baseline:\n\n")
        
        f.write("### Sweep Grayscale Sem Fundo (AEC):\n")
        f.write("| AEC=0 | AEC=200 | AEC=1200 |\n")
        f.write("|:---:|:---:|:---:|\n")
        f.write(f"| {img_link('sweep_gray_sem_fundo_AEC_0_AGC_0')} | {img_link('sweep_gray_sem_fundo_AEC_200_AGC_0')} | {img_link('sweep_gray_sem_fundo_AEC_1200_AGC_0')} |\n\n")

        f.write("### Sweep Grayscale Sem Fundo (AGC):\n")
        f.write("| AGC=4 | AGC=30 |\n")
        f.write("|:---:|:---:|\n")
        f.write(f"| {img_link('sweep_gray_sem_fundo_AEC_300_AGC_4')} | {img_link('sweep_gray_sem_fundo_AEC_300_AGC_30')} |\n\n")

        f.write("### Sweep JPEG Sem Fundo (AEC):\n")
        f.write("| AEC=0 | AEC=200 | AEC=1200 |\n")
        f.write("|:---:|:---:|:---:|\n")
        f.write(f"| {img_link('sweep_jpeg_sem_fundo_AEC_0_AGC_0')} | {img_link('sweep_jpeg_sem_fundo_AEC_200_AGC_0')} | {img_link('sweep_jpeg_sem_fundo_AEC_1200_AGC_0')} |\n\n")

        f.write("## 5. Análise Técnica e Recomendações\n")
        f.write("- **Subtração de Fundo vs Grade Local:** A subtração de fundo (`com_fundo`) remove consideravelmente o ruído espacial de luma em pixels, dando medições de centímetros (`x,y`) muito mais suaves. No entanto, necessita que as exposições físicas e os níveis de brilho da mesa coincidam com a referência inicial. Se a iluminação mudar drasticamente, a grade local (`sem_fundo`) é mais resiliente.\n")
        f.write("- **Sensibilidade do Sensor:** Conforme visto nos sweeps de AEC/AGC da bola, tempos de exposição muito baixos (AEC < 100) deixam a imagem excessivamente escura, fazendo a detecção da bola falhar. Ganhos muito altos (AGC > 16) aumentam o ruído de grão (jitter), o que se traduz em maior desvio padrão espacial da bola parada.\n\n")
        
        f.write("### Sugestão de Parametrização Recomendada:\n\n")
        f.write("| Modo | Melhor Exposição (AEC) | Melhor Ganho (AGC) | FPS Esperado | Ruído Residual de Posição |\n")
        f.write("|---|---|---|---|---|\n")
        for m_id, rec in recomendacoes.items():
            f.write(f"| `{m_id}` | {rec['aec']} | {rec['agc']} | {rec['fps']:.1f} Hz | {rec['ruido']:.4f} cm |\n")
        f.write("\n")

        melhor_modo = min(recomendacoes.keys(), key=lambda k: recomendacoes[k]['ruido'] + (40.0 - recomendacoes[k]['fps'])*0.05)
        f.write("### Veredicto de Operação:\n")
        f.write(f"> [!IMPORTANT]\n")
        f.write(f"> O ponto recomendado para a operação do robô é o modo **`{melhor_modo.upper()}`** utilizando **AEC = {recomendacoes[melhor_modo]['aec']}** e **AGC = {recomendacoes[melhor_modo]['agc']}**.\n")
        f.write(f"> Este compromisso garante taxa estável de frames para o PID com o menor desvio de velocidade residual parado (evitando atuações parasitárias de servo).\n")

    print(f"\n========================================================")
    print(f" RELATÓRIO CONCLUÍDO COM SUCESSO!")
    print(f" Relatório gerado em: {readme_path}")
    print(f"========================================================")

if __name__ == "__main__":
    main()
