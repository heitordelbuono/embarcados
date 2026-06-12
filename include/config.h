#pragma once
// ============================================================
//  config.h  -  TODOS os parametros ajustaveis num lugar so.
//  Mude aqui; nao espalhe numero magico pelo codigo.
// ============================================================

// ---------------- Etapa atual (muda conforme avanca) --------
// 1 = so camera (streaming + FPS)
// 2 = servos
// 3 = reconhecer mesa
// 4 = detectar bola
// 5 = PID fechado
#define ETAPA  4    // 4 = TUDO JUNTO: camera + coordenada + PID + servo manual/danca

// ---------------- Modo da Etapa 1 (camera) ------------------
// 0 = streaming pelo WiFi  (ver ao vivo no navegador)
// 1 = teste SEM WiFi       (mede FPS de captura + manda 1 foto pela serial)
#define MODO_CAMERA  1
#define CAMERA_SWEEP_FPS  0      // 1 = varre resolucoes e mede FPS pela serial
#define CAMERA_SWEEP_SEGUNDOS  4 // tempo por resolucao no sweep

// ---------------- WiFi (necessario na Etapa 1 para streaming) --
#define WIFI_SSID   "Moco24"     // <-- troque aqui
#define WIFI_PASS   "!123456789!"    // <-- troque aqui
#define HTTP_PORT   80

// ---------------- Access Point (Etapas 4/5 — controle pelo celular) --------
// ESP32 cria esta rede. Conecte o celular e abra http://192.168.4.1
#define WIFI_AP_SSID  "MesaPID"   // nome da rede criada pelo ESP32

// ---------------- Camera (OV2640) - PINOS DA ESP-WROVER-KIT ----
// Pinout oficial CAMERA_MODEL_WROVER_KIT da lib esp32-camera.
// (Os pinos da AI-Thinker ESP32-CAM sao DIFERENTES; nao use aqui.)
#define CAM_PIN_PWDN    -1   // WROVER-KIT nao tem pino de power-down
#define CAM_PIN_RESET   -1
#define CAM_PIN_XCLK    21   // <-- na AI-Thinker era 0
#define CAM_PIN_SIOD    26   // SDA da camera (diferente do I2C dos servos)
#define CAM_PIN_SIOC    27   // SCL da camera
#define CAM_PIN_D7      35
#define CAM_PIN_D6      34
#define CAM_PIN_D5      39
#define CAM_PIN_D4      36
#define CAM_PIN_D3      19   // <-- mudou
#define CAM_PIN_D2      18   // <-- mudou
#define CAM_PIN_D1       5   // <-- mudou
#define CAM_PIN_D0       4   // <-- mudou
#define CAM_PIN_VSYNC   25
#define CAM_PIN_HREF    23
#define CAM_PIN_PCLK    22

// Resolucao usada para streaming / FPS (JPEG - mais facil de transmitir)
// Para deteccao (Etapa 4) usamos GRAYSCALE + QQVGA: pixels diretos, sem decodificar JPEG.
#define CAM_STREAM_FRAMESIZE   FRAMESIZE_VGA    // 640x480 para calibracao visual
// Opcoes uteis 4:3: QQVGA 160x120 | QVGA 320x240 | VGA 640x480
// Abaixo de QQVGA o OV2640 normalmente muda proporcao/crop, entao exige recalibrar a mesa.
#define CAM_DETECT_FRAMESIZE   FRAMESIZE_QQVGA  // 160x120 para mirar 20+ FPS on-edge
#define CAM_JPEG_QUALITY       12               // 0=melhor, 63=pior (10-15 e bom)
#define CAM_FB_COUNT           2                // double buffer DMA
#define CAM_XCLK_HZ            20000000         // 20 MHz; teste 24000000 p/ mais FPS

// ---------------- Modo de captura (deteccao) ----------------
// 0 = captura DIRETA (GRAYSCALE/RGB565): leitura pixel a pixel, sem decode.
// 1 = captura em JPEG (clock cheio do sensor = mais FPS) e DECODIFICA on-chip
//     para RGB565. Custa CPU de decode e e com perdas (artefatos), mas serve
//     para medir o ganho de FPS do ponto 3. Mude e recompile para comparar.
#define CAM_CAPTURA_JPEG       1    // 0 = GRAYSCALE direto | 1 = JPEG decodificado (padrao atual)
#define CAM_DETECT_JPEG_Q      10               // qualidade do JPEG na deteccao

// Passos dos ajustes AO VIVO pela serial (ETAPA 4)
#define CAM_AEC_STEP           50               // passo da exposicao (+ / -)
#define CAM_CLK_DIV_INICIAL    4                // divisor DVP inicial p/ teste (menor=+rapido)

// Resolucao numerica (para calculos de deteccao - Etapa 4)
#define CAM_LARGURA        160      // QQVGA
#define CAM_ALTURA         120

// Origem (0,0) no centro da imagem
#define CENTRO_X           (CAM_LARGURA / 2)
#define CENTRO_Y           (CAM_ALTURA  / 2)

// ---------------- Deteccao da bola (amarelo, RGB565) --------
// Componentes RGB565: R 0..31, G 0..63, B 0..31.
// Limiares iniciais p/ amarelo (R alto, G alto, B baixo).
// Serao sobrescritos pela auto-calibracao (salva na NVS).
#define COR_R_MIN          18
#define COR_G_MIN          35
#define COR_B_MAX          14
#define BOLA_MIN_PIXELS    100      // rejeita pontinhos/reflexos; bola atual fica ~600-700 px

// Deteccao inicial da bola branca sobre mesa cinza.
// Y = brilho 0..255; chroma = max(R,G,B)-min(R,G,B).
#define BOLA_Y_MIN         80       // brilho minimo absoluto (baixo p/ cena escura)
#define BOLA_Y_DELTA       20       // quanto mais clara que a media da mesa
#define BOLA_CHROMA_MAX    35       // branco/cinza tem pouca saturacao
#define BOLA_MAX_PIXELS    1300     // rejeita manchas grandes/reflexos em QQVGA
#define BOLA_MIN_LADO      2        // bbox minima em pixels
#define BOLA_MAX_LADO      45       // bbox maxima em pixels
#define BOLA_AREA_ALVO     650      // calibrado pelo debug_bola.ppm atual
#define BOLA_AREA_TOL_PCT  80       // aceita alvo +/-80% (aprox. 130..1170 px)
#define BOLA_ASPECT_MIN_PCT 55      // redondeza por bbox: min(w,h)/max(w,h)
#define BOLA_DENSIDADE_MIN_PCT 35   // preenchimento minimo da bbox
#define BORDA_GRAD_MIN     42       // threshold Sobel p/ debug preto-e-branco
#define VISAO_SCAN_STEP    2        // 1=todo pixel, 2=1/4 dos pixels, mais FPS
#define VISAO_MEDIA_INTERVALO 8     // recalcula brilho medio da mesa a cada N frames

// ---------------- ROI tracking ------------------------------
// Tres controles independentes:
//  VISAO_ROI_LIVRE      1 = varre o FRAME INTEIRO (ignora qualquer poligono)
//  VISAO_ROI_EXTERNO    (so vale se LIVRE=0) 1 = ROI = contorno EXTERNO da mesa
//                       (MESA_EXT, pega a mesa toda) | 0 = ROI interno (MESA_ROI)
//  VISAO_JANELA_DINAMICA 1 = janela de 35% segue a bola (rapido) | 0 = varre o
//                       ROI inteiro todo frame
#define VISAO_ROI_LIVRE       0
#define VISAO_ROI_EXTERNO     1        // ROI = mesa externa toda
#define VISAO_JANELA_DINAMICA 0        // sem janela dinamica
#define ROI_FRACAO         0.35f    // janela = 35% da largura/altura apos achar a bola
#define ROI_PERDE_FRAMES   5        // frames sem achar -> varre tudo

// ---------------- Overlays do frame de debug (tecla 'f') ----
// Liga/desliga cada elemento desenhado sobre a imagem anotada.
#define OVR_MESA           1        // poligono amarelo da mesa externa
#define OVR_ROI            0        // poligono verde do ROI interno (nao limita nada com ROI_EXTERNO=1)
#define OVR_JANELA         1        // retangulo magenta da janela de varredura
#define OVR_EIXOS          1        // cruz amarela no centro da mesa (origem 0,0)
#define OVR_CANDIDATOS     1        // pinta de ciano os pixels candidatos a bola
#define OVR_CAIXA          1        // bounding box vermelha da bola
#define OVR_CENTROIDE      1        // cruz azul no centro (centroide) da bola
#define OVR_MASCARA_PADRAO 0        // 1 = debug 'f' sai preto/branco por padrao

// ---------------- Deteccao avancada (pipeline otimizado) ----
// Modo de cor: 0 = so brilho (grayscale, padrao); 1 = usa cor (RGB565).
// Em grayscale o teste de chroma e desligado (era codigo morto antes).
#define BOLA_MODO_COR        0

// Referencia adaptativa: um pixel e candidato se ficar BOLA_DELTA_REF
// acima da referencia local (grade de iluminacao OU fundo da mesa vazia)
// E acima do piso absoluto BOLA_Y_MIN. Substitui o "media global + delta".
#define BOLA_DELTA_REF       10       // contraste minimo sobre o fundo/grade local

// Grade de iluminacao local: divide a ROI em celulas e usa a media de cada
// celula como referencia -> imune a iluminacao desigual (canto mais claro).
#define GRADE_NX             4
#define GRADE_NY             4

// Subtracao de fundo: capture a mesa VAZIA (comando 'g') e a referencia
// passa a ser o pixel do fundo. Robusto a bola mais clara OU mais escura.
#define FUNDO_ATUALIZA       1        // 1 = media movel lenta do fundo fora da bola
#define FUNDO_ALPHA_SHIFT    6        // peso da media movel (passo = 1/2^n)

// Rotulacao por componentes conexos (run-length + union-find).
// Limites de memoria; overflow -> frame tratado como "nao achei".
#define VISAO_MAX_RUNS       3072     // segmentos horizontais por frame

// Centroide ponderado pela intensidade (sub-pixel) e refino opcional.
#define CENTROIDE_PONDERADO  1        // 1 = peso = (luma - referencia)
#define REFINO_SUBPIXEL      1        // 1 = 2a passada step=1 dentro do bbox

// Filtro alfa-beta: suaviza a posicao e estima a velocidade (cm, cm/s).
// Tambem habilita o gating (rejeita "teleporte") e a predicao da ROI.
#define FILTRO_ATIVO         1
#define FILTRO_GATING_ATIVO  0      // 0 = nao rejeita blob por estar longe do frame anterior
#define FILTRO_ALFA          0.55f    // ganho de posicao (0..1, maior = segue mais)
#define FILTRO_BETA          0.12f    // ganho de velocidade
#define FILTRO_VEL_MAX_CM_S  90.0f    // gating: bola nao se move mais rapido que isso

// ---------------- Camera / exposicao ------------------------
// Padrao atual: auto ligado, porque a cena real esta escura. Se quiser travar
// brilho depois, use 'a' para desligar o AUTO ou ajuste manualmente com + - . ,
#define CAM_AUTO_AJUSTE      1        // 1 = deixa AEC/AGC/AWB no automatico
#define CAM_AEC_FIXO         300      // exposicao fixa (0..1200)
#define CAM_AGC_FIXO         0        // ganho fixo (0..30)
#define CAM_CALIBRA_MS       1200     // tempo com auto ligado antes de congelar
#define CAM_USA_NVS          1        // 1 = salva/le a calibracao da camera na NVS

// Mesa calibrada na imagem VGA 640x480. Pontos externos podem cair fora
// do frame porque a camera nao ve a mesa inteira; as retas sao extrapoladas.
// O firmware escala estes pontos automaticamente para CAM_LARGURA/CAM_ALTURA.
#define MESA_LADO_CM       19.0f
#define MESA_CALIB_LARGURA 640.0f
#define MESA_CALIB_ALTURA  480.0f
#define MESA_EXT_TL_X      37
#define MESA_EXT_TL_Y      29
#define MESA_EXT_TR_X      542
#define MESA_EXT_TR_Y      -34
#define MESA_EXT_BR_X      605
#define MESA_EXT_BR_Y      478
#define MESA_EXT_BL_X      99
#define MESA_EXT_BL_Y      542

// ROI quase toda dentro da mesa, tambem extrapolada se necessario.
#define MESA_ROI_TL_X      72
#define MESA_ROI_TL_Y      78
#define MESA_ROI_TR_X      520
#define MESA_ROI_TR_Y      23
#define MESA_ROI_BR_X      568
#define MESA_ROI_BR_Y      415
#define MESA_ROI_BL_X      120
#define MESA_ROI_BL_Y      470

// ---------------- Controle PID (valores do controlePID.m) ---
#define PID_KP             0.0f     // comeca em 0; sobe na mao com P/p
#define PID_KD             0.0f     // comeca em 0; sobe na mao com D/d
#define PID_KI             0.0f     // comeca em 0; sobe na mao com I/i
#define PID_I_MAX          25.0f    // anti-windup: limite do termo integral (graus)
// Escala da saida do PID: a soma (Kp*e + Ki*i + Kd*d) e MULTIPLICADA por isto
// antes de saturar em +-SERVO_RANGE. 0.1 = ganhos efetivos 10x menores -> voce
// tuna com numeros 10x maiores (passo de Kp mais fino).
#define PID_ESCALA_SAIDA   0.1f
#define PID_TAU            0.01f    // = Td/N (filtro da derivada)
#define PID_DT             0.02f    // 50 Hz
// Setpoint padrao (centro). Mudavel por Bluetooth.
#define SETPOINT_X_PADRAO  0.0f
#define SETPOINT_Y_PADRAO  0.0f
// Atraso de atuacao agendado na fila de eventos (us)
#define ATUACAO_ATRASO_US  20000    // 20 ms

// ---------------- Servos / PCA9685 --------------------------
// O PCA9685 COMPARTILHA o barramento I2C da camera (SCCB), nos pinos SIOD/SIOC.
// Motivo: nesta placa o 32/33 conflita com a camera, e nao sobra par de pino
// livre. A camera ja poe pull-up e cria o bus no port 1; o servo so se pendura.
// >>> Ligue SDA do PCA no GPIO 26 e SCL no GPIO 27 (mesmos pinos da camera). <<<
#define CAM_SCCB_PORT      1        // port I2C que a camera usa (CONFIG_SCCB_HARDWARE_I2C_PORT1)
#define I2C_SDA_GPIO       26       // = CAM_PIN_SIOD (so p/ log; o bus e o da camera)
#define I2C_SCL_GPIO       27       // = CAM_PIN_SIOC
#define I2C_FREQ_HZ        100000   // velocidade do device PCA no bus compartilhado
#define PCA9685_ENDERECO   0x40
#define PCA9685_FREQ_HZ    50       // 50 Hz p/ servo

#define SERVO_CANAL_X      1        // trocado: eixo X da imagem -> servo no canal 1
#define SERVO_CANAL_Y      0        // trocado: eixo Y da imagem -> servo no canal 0
#define SERVO_NEUTRO_X     91       // angulo real quando mesa esta plana (eixo X) — era 97, -6 graus
#define SERVO_NEUTRO_Y     97       // angulo real quando mesa esta plana (eixo Y)
#define SERVO_X_INVERTIDO  1        // 1 = inverte direcao do servo X
#define SERVO_Y_INVERTIDO  0        // 1 = inverte direcao do servo Y
#define SERVO_RANGE        30       // desvio maximo a partir do neutro (+/- graus)
#define SERVO_DANCE_RANGE  10       // desvio maximo da danca (+/- graus)
#define SERVO_DANCE_SPEED_X 3.5f    // velocidade angular da gangorra no eixo X
#define SERVO_DANCE_SPEED_Y 3.5f    // velocidade angular da gangorra no eixo Y
#define SERVO_DANCE_HOLD_MS 500     // pausa em cada posicao aleatoria (ms)
#define SERVO_MIN          0        // limite fisico minimo
#define SERVO_MAX          180      // limite fisico maximo
#define SERVO_PULSO_MIN_US 500
#define SERVO_PULSO_MAX_US 2500

// ---------------- FreeRTOS (dual-core) ----------------------
#define NUCLEO_VISAO       1
#define NUCLEO_CONTROLE    1
#define NUCLEO_COMMS       0
#define PRIO_VISAO         5
#define PRIO_CONTROLE      6
#define PRIO_COMMS         3
