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
#define ETAPA  1

// ---------------- Modo da Etapa 1 (camera) ------------------
// 0 = streaming pelo WiFi  (ver ao vivo no navegador)
// 1 = teste SEM WiFi       (mede FPS de captura + manda 1 foto pela serial)
#define MODO_CAMERA  0
#define CAMERA_SWEEP_FPS  1      // 1 = varre resolucoes e mede FPS pela serial
#define CAMERA_SWEEP_SEGUNDOS  4 // tempo por resolucao no sweep

// ---------------- WiFi (necessario na Etapa 1 para streaming) --
#define WIFI_SSID   "Moco24"     // <-- troque aqui
#define WIFI_PASS   "!123456789!"    // <-- troque aqui
#define HTTP_PORT   80

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
// Para deteccao (Etapa 4) vai mudar para RGB565 + QQVGA
#define CAM_STREAM_FRAMESIZE   FRAMESIZE_CIF    // 400x296 para stream WiFi rapido
// Opcoes: QQVGA 160x120 | QVGA 320x240 | VGA 640x480 | SVGA 800x600 | XGA 1024x768
#define CAM_DETECT_FRAMESIZE   FRAMESIZE_QQVGA  // 160x120 para deteccao rapida
#define CAM_JPEG_QUALITY       12               // 0=melhor, 63=pior (10-15 e bom)
#define CAM_FB_COUNT           2                // double buffer DMA
#define CAM_XCLK_HZ            20000000

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
#define BOLA_MIN_PIXELS    8        // menos que isso = "nao achei"

// ---------------- ROI tracking ------------------------------
#define ROI_FRACAO         0.30f    // janela = 30% da largura/altura
#define ROI_PERDE_FRAMES   5        // frames sem achar -> varre tudo

// ---------------- Controle PID (valores do controlePID.m) ---
#define PID_KP             200.0f
#define PID_KD             1.0f     // = Kp*Td
#define PID_TAU            0.01f    // = Td/N (filtro da derivada)
#define PID_DT             0.02f    // 50 Hz
// Setpoint padrao (centro). Mudavel por Bluetooth.
#define SETPOINT_X_PADRAO  0.0f
#define SETPOINT_Y_PADRAO  0.0f
// Atraso de atuacao agendado na fila de eventos (us)
#define ATUACAO_ATRASO_US  20000    // 20 ms

// ---------------- Servos / PCA9685 --------------------------
#define I2C_SDA_GPIO       14
#define I2C_SCL_GPIO       15
#define I2C_FREQ_HZ        400000
#define PCA9685_ENDERECO   0x40
#define PCA9685_FREQ_HZ    50       // 50 Hz p/ servo

#define SERVO_CANAL_X      0
#define SERVO_CANAL_Y      1
#define SERVO_NEUTRO_X     100      // angulo real quando mesa esta plana (eixo X)
#define SERVO_NEUTRO_Y     95       // angulo real quando mesa esta plana (eixo Y)
#define SERVO_X_INVERTIDO  1        // 1 = inverte direcao do servo X
#define SERVO_RANGE        20       // desvio maximo a partir do neutro (+/- graus)
#define SERVO_DANCE_RANGE  10       // desvio maximo da danca (+/- graus)
#define SERVO_DANCE_SPEED  0.4f     // graus por tick (tick=20ms) -> ~20 graus/s
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
