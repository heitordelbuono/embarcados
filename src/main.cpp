// ============================================================
//  main.cpp - Mesa Balanceadora PID
//  Etapa atual definida em include/config.h (#define ETAPA N)
// ============================================================
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_camera.h"
#include "config.h"
#include "tipos.h"
#include "visao.h"
#include "maquina_estados.h"

static const char* TAG = "MESA";

// ============================================================
//  ETAPA 1 - so camera: inicia, conecta WiFi, serve stream
// ============================================================
#if ETAPA == 1

#include "wifi_stream.h"
#include "camera_teste.h"

extern "C" void app_main(void) {
    ESP_LOGI(TAG, "Mesa Balanceadora - ETAPA 1 (camera)");

    Visao visao;
    if (!visao.inicia()) {
        ESP_LOGE(TAG, "Camera nao iniciou. Verifique o hardware e reinicie.");
        return;
    }

#if MODO_CAMERA == 0
    // Streaming pelo WiFi: imprime o IP na serial, abra no navegador.
    wifi_stream_inicia();
    for (;;) vTaskDelay(pdMS_TO_TICKS(10000));
#else
    // Teste SEM WiFi: foto pela serial + FPS de captura. Nao retorna.
    camera_teste_benchmark();
#endif
}

// ============================================================
//  ETAPA 2 - teste dos servos via PCA9685
// ============================================================
#elif ETAPA == 2

#include "driver_atuacao.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <fcntl.h>

static DriverAtuacao driver;

static void gangorra(bool usa_x, bool usa_y) {
    printf("Modo gangorra %s (+-%.0f graus). Enter para parar.\n",
           usa_x && usa_y ? "X+Y" : (usa_x ? "X" : "Y"),
           (float)SERVO_DANCE_RANGE);

    // stdin nao-bloqueante so durante a danca
    int sflags = fcntl(fileno(stdin), F_GETFL, 0);
    fcntl(fileno(stdin), F_SETFL, sflags | O_NONBLOCK);
    while (getchar() != EOF) {}  // drena o \n do proprio comando

    float fase_x = 0.0f;
    float fase_y = 0.0f;
    float amplitude = 0.0f;
    bool parar = false;
    while (!parar) {
        if (amplitude < SERVO_DANCE_RANGE) {
            amplitude += 0.08f;
            if (amplitude > SERVO_DANCE_RANGE) amplitude = SERVO_DANCE_RANGE;
        }

        float sinal_x = sinf(fase_x) * amplitude;
        float sinal_y = sinf(fase_y) * amplitude;
        driver.enviaCorrecaoX(usa_x ? sinal_x : 0.0f);
        driver.enviaCorrecaoY(usa_y ? sinal_y : 0.0f);

        fase_x += SERVO_DANCE_SPEED_X * 0.03f;
        fase_y += SERVO_DANCE_SPEED_Y * 0.03f;
        if (fase_x >= 2.0f * (float)M_PI) fase_x -= 2.0f * (float)M_PI;
        if (fase_y >= 2.0f * (float)M_PI) fase_y -= 2.0f * (float)M_PI;

        vTaskDelay(pdMS_TO_TICKS(20));
        int c2 = getchar();  // retorna -1 imediatamente se nao ha tecla
        if (c2 == '\n' || c2 == '\r') parar = true;
    }

    fcntl(fileno(stdin), F_SETFL, sflags);
    driver.neutro();
    printf("Danca parada. Neutro.\n");
}

static void ajuda() {
    printf("\n=== CONTROLE DE SERVOS ===\n");
    printf("  x<sinal>  -> servo X, -%d..+%d graus  (ex: x-10  x15)\n", SERVO_RANGE, SERVO_RANGE);
    printf("  y<sinal>  -> servo Y, -%d..+%d graus  (ex: y-10  y15)\n", SERVO_RANGE, SERVO_RANGE);
    printf("  p<canal>,<us> -> pulso direto em us  (ex: p0,1500)\n");
    printf("  n         -> neutro (mesa plana)\n");
    printf("  s         -> varredura: -%d -> 0 -> +%d -> 0\n", SERVO_RANGE, SERVO_RANGE);
    printf("  a         -> gangorra no eixo X (Enter para parar)\n");
    printf("  b         -> gangorra no eixo Y (Enter para parar)\n");
    printf("  ab        -> gangorra nos eixos X+Y (Enter para parar)\n");
    printf("  d         -> debug: I2C scan + registradores PCA9685\n");
    printf("  ?         -> esta ajuda\n");
    printf("  Neutros: X=%d  Y=%d  |  Range: +/-%d  |  X %s\n",
           SERVO_NEUTRO_X, SERVO_NEUTRO_Y, SERVO_RANGE,
           SERVO_X_INVERTIDO ? "(invertido)" : "(normal)");
    printf("==========================\n\n");
}

extern "C" void app_main(void) {
    ESP_LOGI(TAG, "Mesa Balanceadora - ETAPA 2 (servos interativo)");
    vTaskDelay(pdMS_TO_TICKS(500));

    driver.iniciaMotores();
    vTaskDelay(pdMS_TO_TICKS(200));

    ajuda();

    char buf[32];
    int  pos = 0;

    for (;;) {
        int c = getchar();
        if (c < 0) { vTaskDelay(pdMS_TO_TICKS(10)); continue; }

        if (c == '\n' || c == '\r') {
            buf[pos] = '\0';
            if (pos == 0) { pos = 0; continue; }

            char cmd = buf[0];
            float ang = (pos > 1) ? (float)atof(buf + 1) : 0.0f;

            if (cmd == 'x' || cmd == 'X') {
                printf("Servo X sinal=%.1f (neutro=%d %s)\n", ang,
                       SERVO_NEUTRO_X, SERVO_X_INVERTIDO ? "inv" : "");
                driver.enviaCorrecaoX(ang);
            } else if (cmd == 'y' || cmd == 'Y') {
                printf("Servo Y sinal=%.1f (neutro=%d)\n", ang, SERVO_NEUTRO_Y);
                driver.enviaCorrecaoY(ang);
            } else if (cmd == 'n' || cmd == 'N') {
                printf("Neutro (mesa plana)\n");
                driver.neutro();
            } else if (cmd == 's' || cmd == 'S') {
                const float seq[] = { -SERVO_RANGE, 0, SERVO_RANGE, 0 };
                const char* lab[] = { "-90", "0 (neutro)", "+90", "0 (neutro)" };
                for (int i = 0; i < 4; i++) {
                    printf("X+Y sinal=%s (%.0f)\n", lab[i], seq[i]);
                    driver.enviaCorrecaoX(seq[i]);
                    driver.enviaCorrecaoY(seq[i]);
                    vTaskDelay(pdMS_TO_TICKS(1500));
                }
            } else if ((buf[0] == 'a' || buf[0] == 'A') &&
                       (buf[1] == 'b' || buf[1] == 'B') &&
                       buf[2] == '\0') {
                gangorra(true, true);
            } else if (cmd == 'a' || cmd == 'A') {
                gangorra(true, false);
            } else if (cmd == 'b' || cmd == 'B') {
                gangorra(false, true);
            } else if (cmd == 'd' || cmd == 'D') {
                driver.debug();
            } else if (cmd == 'p' || cmd == 'P') {
                // p<canal>,<us>  ex: p0,1500
                int canal = 0; int us = 1500;
                sscanf(buf + 1, "%d,%d", &canal, &us);
                printf("Pulso direto: canal %d, %d us\n", canal, us);
                driver.escrevePulso(canal, (uint16_t)us);
            } else if (cmd == '?') {
                ajuda();
            } else {
                printf("Comando desconhecido: '%s'\n", buf);
            }
            pos = 0;
        } else if (pos < 31) {
            buf[pos++] = (char)c;
        }
    }
}

// ============================================================
//  ETAPA 0 - TESTE MINIMO DO SERVO (PCA9685 via I2C, SDA=14 SCL=15)
//  Sem camera, sem PID, sem nada. So fala com o PCA9685 e varre o
//  servo dos canais 0 e 1 pra sempre. Use isto pra isolar o hardware.
// ============================================================
#elif ETAPA == 0

#include <stdio.h>
#include "driver/i2c_master.h"

#define T_ADDR  0x40
#define T_FREQ  50            // Hz do servo

static i2c_master_bus_handle_t t_bus = NULL;
static i2c_master_dev_handle_t t_dev = NULL;

static void t_w(uint8_t reg, uint8_t val) {              // escreve 1 registrador
    uint8_t b[2] = {reg, val};
    esp_err_t e = i2c_master_transmit(t_dev, b, 2, 100);
    if (e != ESP_OK) printf("  ERRO escrita reg 0x%02X: %s\n", reg, esp_err_to_name(e));
}

static void t_pulso(int canal, int us) {                 // manda pulso em us
    uint16_t off = (uint16_t)((float)us / (1000000.0f / T_FREQ) * 4096.0f);
    uint8_t reg = 0x06 + 4 * canal;                      // LED0_ON_L + 4*canal
    uint8_t b[5] = {reg, 0, 0, (uint8_t)(off & 0xFF), (uint8_t)(off >> 8)};
    esp_err_t e = i2c_master_transmit(t_dev, b, 5, 100);
    printf("  canal %d -> %d us (off=%u) %s\n", canal, us, off,
           e == ESP_OK ? "OK" : esp_err_to_name(e));
}

// cria o barramento numa orientacao, escaneia, e devolve o 1o endereco que
// deu ACK (ou -1). Limpa o barramento no fim (a menos que ache algo).
static int scan_em(int sda, int scl, bool manter) {
    i2c_master_bus_handle_t bus = NULL;
    i2c_master_bus_config_t bc = {};
    bc.i2c_port          = I2C_NUM_0;
    bc.sda_io_num        = (gpio_num_t)sda;
    bc.scl_io_num        = (gpio_num_t)scl;
    bc.clk_source        = I2C_CLK_SRC_DEFAULT;
    bc.glitch_ignore_cnt = 7;
    bc.flags.enable_internal_pullup = true;
    if (i2c_new_master_bus(&bc, &bus) != ESP_OK) {
        printf("-- nao consegui criar bus SDA=%d SCL=%d --\n", sda, scl);
        return -1;
    }
    printf("-- scan com SDA=%d SCL=%d --\n", sda, scl);
    int achou = -1;
    for (uint8_t a = 0x08; a <= 0x77; a++) {
        if (i2c_master_probe(bus, a, 100) == ESP_OK) {
            printf("  [ACK] 0x%02X%s\n", a, a == T_ADDR ? "  <- PCA9685" : "");
            if (achou < 0) achou = a;
        }
    }
    if (achou < 0) printf("  (nada respondeu nesta orientacao)\n");
    if (achou >= 0 && manter) { t_bus = bus; }   // mantem o bus que funcionou
    else                      { i2c_del_master_bus(bus); }
    return achou;
}

extern "C" void app_main(void) {
    vTaskDelay(pdMS_TO_TICKS(500));
    printf("\n=== TESTE PCA9685: tenta as DUAS orientacoes de SDA/SCL ===\n");

    // Tenta primeiro 32/33; se nada, tenta os fios trocados (33/32).
    int sda_ok = 32, scl_ok = 33;
    int f = scan_em(32, 33, true);
    if (f < 0) { sda_ok = 33; scl_ok = 32; f = scan_em(33, 32, true); }

    if (f < 0) {
        printf("\n>> NINGUEM respondeu em NENHUMA orientacao.\n");
        printf(">> Logo NAO e SDA/SCL trocado nem endereco -> e ENERGIA ou conexao:\n");
        printf(">>   1) VCC do modulo no 3.3V (o LED vermelho do PCA acende?)\n");
        printf(">>   2) GND do PCA ligado no GND do ESP (comum)\n");
        printf(">>   3) os 2 fios bem encaixados em 32 e 33 e nos pinos SDA/SCL do PCA\n");
        return;
    }

    printf("\n>> ACHOU 0x%02X com SDA=%d SCL=%d <<\n", f, sda_ok, scl_ok);
    if (sda_ok != 32) printf(">> ATENCAO: os fios estao TROCADOS! Configure I2C_SDA_GPIO=%d I2C_SCL_GPIO=%d\n", sda_ok, scl_ok);

    // configura o dispositivo no bus que funcionou
    i2c_device_config_t dc = {};
    dc.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dc.device_address  = T_ADDR;
    dc.scl_speed_hz    = 100000;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(t_bus, &dc, &t_dev));

    // configura o PCA9685 pra 50 Hz
    t_w(0x00, 0x00);                                     // MODE1 = normal
    vTaskDelay(pdMS_TO_TICKS(5));
    uint8_t prescale = (uint8_t)(25000000UL / (4096UL * T_FREQ) - 1);  // ~121
    t_w(0x00, 0x10);                                     // sleep p/ mexer no prescale
    t_w(0xFE, prescale);                                 // PRESCALE
    t_w(0x00, 0x00);                                     // acorda
    vTaskDelay(pdMS_TO_TICKS(5));
    t_w(0x00, 0xA0);                                     // restart + auto-increment
    printf("PCA9685 configurado: prescale=%u (~%d Hz). Varrendo canais 0 e 1...\n", prescale, T_FREQ);

    // varre pra sempre: 1.0ms -> 1.5ms -> 2.0ms -> 1.5ms
    for (;;) {
        printf("-- min --\n");    t_pulso(0, 1000); t_pulso(1, 1000); vTaskDelay(pdMS_TO_TICKS(1000));
        printf("-- meio --\n");   t_pulso(0, 1500); t_pulso(1, 1500); vTaskDelay(pdMS_TO_TICKS(1000));
        printf("-- max --\n");    t_pulso(0, 2000); t_pulso(1, 2000); vTaskDelay(pdMS_TO_TICKS(1000));
        printf("-- meio --\n");   t_pulso(0, 1500); t_pulso(1, 1500); vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// ============================================================
//  ETAPA 9 - SERVO DIRETO por PWM (LEDC), SEM PCA9685, SEM I2C.
//  Liga o fio de SINAL do servo direto no GPIO. Sem pull-up, sem nada.
//  Servo 1 -> GPIO 32 | Servo 2 -> GPIO 33 | 5V e GND do servo a parte,
//  com GND COMUM no ESP. Se mexer aqui, o problema era o PCA9685/I2C.
// ============================================================
#elif ETAPA == 9

#include <stdio.h>
#include "driver/ledc.h"

#define S1_GPIO     32
#define S2_GPIO     33
#define S_FREQ      50                 // Hz do servo
#define S_RES       LEDC_TIMER_16_BIT  // 0..65535 numa janela de 20 ms

// converte largura de pulso (us) em duty de 16 bits (periodo = 20000 us)
static uint32_t us2duty(int us) {
    return (uint32_t)((float)us / 20000.0f * 65536.0f);
}

static void servo_us(ledc_channel_t ch, int us) {
    ledc_set_duty(LEDC_LOW_SPEED_MODE, ch, us2duty(us));
    ledc_update_duty(LEDC_LOW_SPEED_MODE, ch);
    printf("  ch%d -> %d us (duty=%lu)\n", (int)ch, us, (unsigned long)us2duty(us));
}

extern "C" void app_main(void) {
    vTaskDelay(pdMS_TO_TICKS(500));
    printf("\n=== SERVO DIRETO (LEDC, SEM PCA9685) S1=GPIO%d S2=GPIO%d ===\n", S1_GPIO, S2_GPIO);

    // 1) timer LEDC a 50 Hz
    ledc_timer_config_t tcfg = {};
    tcfg.speed_mode      = LEDC_LOW_SPEED_MODE;
    tcfg.timer_num       = LEDC_TIMER_0;
    tcfg.duty_resolution = S_RES;
    tcfg.freq_hz         = S_FREQ;
    tcfg.clk_cfg         = LEDC_AUTO_CLK;
    ESP_ERROR_CHECK(ledc_timer_config(&tcfg));

    // 2) dois canais, um por GPIO
    ledc_channel_config_t ccfg = {};
    ccfg.speed_mode = LEDC_LOW_SPEED_MODE;
    ccfg.timer_sel  = LEDC_TIMER_0;
    ccfg.intr_type  = LEDC_INTR_DISABLE;
    ccfg.hpoint     = 0;
    ccfg.duty       = 0;

    ccfg.gpio_num = S1_GPIO; ccfg.channel = LEDC_CHANNEL_0; ESP_ERROR_CHECK(ledc_channel_config(&ccfg));
    ccfg.gpio_num = S2_GPIO; ccfg.channel = LEDC_CHANNEL_1; ESP_ERROR_CHECK(ledc_channel_config(&ccfg));

    printf("LEDC pronto. Varrendo os dois servos...\n");

    // 3) varre pra sempre: 1.0 -> 1.5 -> 2.0 -> 1.5 ms
    for (;;) {
        printf("-- min (1.0ms) --\n");  servo_us(LEDC_CHANNEL_0, 1000); servo_us(LEDC_CHANNEL_1, 1000); vTaskDelay(pdMS_TO_TICKS(1000));
        printf("-- meio (1.5ms) --\n"); servo_us(LEDC_CHANNEL_0, 1500); servo_us(LEDC_CHANNEL_1, 1500); vTaskDelay(pdMS_TO_TICKS(1000));
        printf("-- max (2.0ms) --\n");  servo_us(LEDC_CHANNEL_0, 2000); servo_us(LEDC_CHANNEL_1, 2000); vTaskDelay(pdMS_TO_TICKS(1000));
        printf("-- meio (1.5ms) --\n"); servo_us(LEDC_CHANNEL_0, 1500); servo_us(LEDC_CHANNEL_1, 1500); vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// ============================================================
//  ETAPA 4 - detectar bola branca on-edge + debug serial
// ============================================================
#elif ETAPA == 4

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <fcntl.h>
#include "gerenciador.h"
#include "wifi_controle.h"

static ModuloGerenciador g;
static bool pidX = false, pidY = false;   // PID por eixo (ox/oy/o)

// Mede fps/cap/proc rodando detecta() por 'seg' segundos.
static void mede_fps(Visao& visao, float seg, float& fps, float& cap_ms, float& proc_ms) {
    int64_t t0 = esp_timer_get_time();
    int fr = 0; int64_t scap = 0, sproc = 0;
    while ((esp_timer_get_time() - t0) < (int64_t)(seg * 1e6f)) {
        visao.detecta();
        const VisaoDebugInfo& d = visao.debugInfo();
        scap += d.captura_us; sproc += d.processo_us; fr++;
    }
    int64_t dt = esp_timer_get_time() - t0;
    fps    = dt ? fr * 1e6f / (float)dt : 0;
    cap_ms = fr ? scap / (float)fr / 1000.0f : 0;
    proc_ms= fr ? sproc / (float)fr / 1000.0f : 0;
}

// Sweep automatico: varre exposicao (aec) e ganho (agc), medindo o FPS de
// cada ponto e imprimindo CSV. Serve p/ os dois modos de captura.
static void sweep_sensor(Visao& visao) {
    const int aec_list[] = { 0, 100, 200, 400, 800, 1200 };
    const int agc_list[] = { 0, 4, 8, 16, 30 };
    const int n_aec = sizeof(aec_list) / sizeof(aec_list[0]);
    const int n_agc = sizeof(agc_list) / sizeof(agc_list[0]);

    printf("===SWEEP_INICIO modo=%s===\n", CAM_CAPTURA_JPEG ? "JPEG" : "GRAY");
    printf("SWEEP,fase,aec,agc,clkdiv,fps,cap_ms,proc_ms\n");

    float fps, cap, proc;

    // Fase 1: varre EXPOSICAO (ganho fixo)
    for (int i = 0; i < n_aec; i++) {
        visao.configuraSensor(aec_list[i], CAM_AGC_FIXO, -1);
        for (int k = 0; k < 8; k++) visao.detecta();          // estabiliza
        mede_fps(visao, 1.5f, fps, cap, proc);
        printf("SWEEP,AEC,%d,%d,-1,%.1f,%.1f,%.1f\n", aec_list[i], CAM_AGC_FIXO, fps, cap, proc);
    }

    // Fase 2: varre GANHO (exposicao fixa)
    for (int i = 0; i < n_agc; i++) {
        visao.configuraSensor(CAM_AEC_FIXO, agc_list[i], -1);
        for (int k = 0; k < 8; k++) visao.detecta();
        mede_fps(visao, 1.5f, fps, cap, proc);
        printf("SWEEP,AGC,%d,%d,-1,%.1f,%.1f,%.1f\n", CAM_AEC_FIXO, agc_list[i], fps, cap, proc);
    }

    printf("===SWEEP_FIM===\n");
    visao.configuraSensor(CAM_AEC_FIXO, CAM_AGC_FIXO, -1);     // volta ao baseline
}

// Loop principal (camera + PID + serial) numa task do CORE 1, longe do WiFi.
// O WiFi/HTTP ficam no core 0; rodar isto no app_main (core 0) derrubava o
// FPS e estourava o watchdog do IDLE0.
static void tarefa_principal(void* arg) {
    int sflags = fcntl(fileno(stdin), F_GETFL, 0);
    fcntl(fileno(stdin), F_SETFL, sflags | O_NONBLOCK);

    auto ajuda = [&]() {
        printf("\n=== MESA PID (modo normal de execucao) ===\n");
        printf(" Deteccao/coordenada:\n");
        printf("  f -> frame anotado pela serial (imagem original + overlays)\n");
        printf("  g -> captura o FUNDO (mesa VAZIA) -> ativa subtracao de fundo\n");
        printf("  G -> descarta o fundo (volta para grade de iluminacao)\n");
        printf("  e -> liga/desliga overlay Sobel no frame de debug\n");
        printf("  h -> liga/desliga debug de mascara (branco=candidato, preto=resto)\n");
        printf("  r -> reinicia o filtro alfa-beta\n");
        printf("  t -> liga/desliga telemetria CSV (t,x,y,vx,vy,achou,fps)\n");
        printf(" Captura / sensor / FPS:\n");
        printf("  j     -> ALTERNA captura GRAYSCALE <-> JPEG (em runtime)\n");
        printf("  + / - -> exposicao (aec) +/- %d  (menor = +FPS, mais escuro)\n", CAM_AEC_STEP);
        printf("  . / , -> ganho (agc) +/- 1       (clareia sem custar FPS, +ruido)\n");
        printf("  a     -> liga/desliga AUTO (exposicao/ganho/AWB)\n");
        printf("  k / l -> divisor de clock DVP - / +  (manual; pode travar -> reboot)\n");
        printf("  c     -> calibra a camera (auto -> congela -> salva NVS)\n");
        printf("  m     -> calibra geometria da mesa (detecta cantos e homografia)\n");
        printf("  0cx<n>/0cy<n> -> desloca centro em passos de 0.1 cm (ex: 0cx1 0cy-2)\n");
        printf("  ux<n> / uy<n> -> desloca centro/origem em pixels (ex: ux2 uy-1)\n");
        printf("  us / u0       -> salva / reseta centro manual (0s salva tudo, 00 reseta tudo)\n");
        printf("  q     -> imprime estado do sensor (aec/agc/auto/clkdiv) e ganhos PID\n");
        printf("  S     -> SWEEP automatico (varre exposicao+ganho, imprime CSV)\n");
        printf(" Servo na mao (testa hardware, coordenada continua rodando):\n");
        printf("  x<val> -> servo X manual, -%d..+%d graus + Enter (ex: x-10  x15)\n", SERVO_RANGE, SERVO_RANGE);
        printf("  y<val> -> servo Y manual, -%d..+%d graus + Enter (ex: y-10  y15)\n", SERVO_RANGE, SERVO_RANGE);
        printf("  w     -> liga/desliga DANCA no eixo X (gangorra senoidal)\n");
        printf("  b     -> liga/desliga DANCA no eixo Y  (w+b = X+Y juntos)\n");
        printf("  n     -> NEUTRO: para manual/danca/PID e zera a mesa\n");
        printf("  T     -> TROCA os canais dos servos X<->Y (na hora)\n");
        printf("  0x<n> / 0y<n> -> ajusta zero do servo em graus (ex: 0x1 0y-1)\n");
        printf("  zx<n> / zy<n> -> ajusta neutro do servo em graus (ex: zx1 zy-0.5)\n");
        printf("  zs / z0       -> salva / reseta neutro dos servos\n");
        printf(" Controle / PID (precisa Enter):\n");
        printf("  o  -> liga/desliga PID nos DOIS eixos\n");
        printf("  ox -> PID so no eixo X (Y fica plano)  |  oy -> so no eixo Y\n");
        printf("  P / p -> Aumentar/Diminuir Proporcional (Kp) em 1\n");
        printf("  I / i -> Aumentar/Diminuir Integral (Ki) em 0.1\n");
        printf("  D / d -> Aumentar/Diminuir Derivativo (Kd) em 0.1\n");
        printf("  s     -> Varredura rapida dos servos (-10 -> 0 -> +10 -> 0)\n");
        printf("  v     -> Depuracao dos servos (I2C scan + registradores)\n");
        printf("  ?     -> esta ajuda\n");
        printf("Modo atual: %s | print a cada 5s\n\n",
               g.visao.modoJpeg() ? "JPEG" : (BOLA_MODO_COR ? "RGB565" : "GRAYSCALE"));
    };
    ajuda();

    int frames = 0;
    int achou_frames = 0;
    int atuou = 0;            // quantas vezes o PID atuou (com bola) na janela
    bool sobel = false;
    bool mascara = false;
    bool telemetria = false;
    bool autoSens = (CAM_AUTO_AJUSTE != 0);
    int64_t t0 = esp_timer_get_time();
    float fps_atual = 0.0f;

    // --- controle manual / danca dos servos (rodam SEM bloquear a camera) ---
    bool  manualAtivo = false;            // segura um angulo fixo (comandos x/y)
    float manualX = 0.0f, manualY = 0.0f;
    bool  dancaX = false, dancaY = false; // gangorra senoidal por eixo (w/b)
    float danceFaseX = 0.0f, danceFaseY = 0.0f, danceAmp = 0.0f;
    // buffer de linha p/ comandos com argumento (x<val>, y<val>) + Enter
    char  linebuf[16];
    int   linepos = 0;
    bool  capturando = false;

    for (;;) {
        Medicao m = g.visao.detecta();
        const VisaoDebugInfo& d = g.visao.debugInfo();
        frames++;
        if (m.achou) achou_frames++;

        // Atualiza posicao para o servidor HTTP
        g_pos_web.x     = m.x;
        g_pos_web.y     = m.y;
        g_pos_web.achou = m.achou;

        // Aplica setpoint recebido pelo browser (clique na mesa)
        g.setpointX = g_setpoint_x;
        g.setpointY = g_setpoint_y;

        // Prioridade de atuacao: PID > danca > manual > neutro
        if (pidX || pidY) {
            g.calculaAcaoControle(m, pidX, pidY);
            if (m.achou) atuou++;          // contou uma atuacao de fato (tinha bola)
        } else if (dancaX || dancaY) {
            if (danceAmp < SERVO_DANCE_RANGE) {
                danceAmp += 0.08f;
                if (danceAmp > SERVO_DANCE_RANGE) danceAmp = SERVO_DANCE_RANGE;
            }
            float sx = sinf(danceFaseX) * danceAmp;
            float sy = sinf(danceFaseY) * danceAmp;
            g.driver.enviaCorrecaoX(dancaX ? sx : 0.0f);
            g.driver.enviaCorrecaoY(dancaY ? sy : 0.0f);
            danceFaseX += SERVO_DANCE_SPEED_X * 0.03f;
            danceFaseY += SERVO_DANCE_SPEED_Y * 0.03f;
            if (danceFaseX >= 2.0f * (float)M_PI) danceFaseX -= 2.0f * (float)M_PI;
            if (danceFaseY >= 2.0f * (float)M_PI) danceFaseY -= 2.0f * (float)M_PI;
        } else if (manualAtivo) {
            g.driver.enviaCorrecaoX(manualX);
            g.driver.enviaCorrecaoY(manualY);
        } else {
            g.driver.neutro();
        }
        g.processaFila();

        if (telemetria) {
            printf("CSV,%lld,%.3f,%.3f,%.3f,%.3f,%d,%.1f\n",
                   (long long)m.t_us, m.x, m.y, m.vx, m.vy, m.achou ? 1 : 0, fps_atual);
        }

        int c;
        while ((c = getchar()) >= 0) {
            // --- comandos com argumento terminados por Enter ---
            if (capturando) {
                if (c == '\n' || c == '\r') {
                    linebuf[linepos] = '\0';
                    char cmd = linebuf[0];
                    if (cmd == 'o' || cmd == 'O') {
                        // o = alterna os 2 eixos | ox = so X | oy = so Y
                        char ax = (linepos > 1) ? linebuf[1] : '\0';
                        dancaX = dancaY = false; manualAtivo = false;
                        if      (ax == 'x' || ax == 'X') { pidX = true;  pidY = false; }
                        else if (ax == 'y' || ax == 'Y') { pidY = true;  pidX = false; }
                        else { bool on = !(pidX || pidY); pidX = pidY = on; }  // 'o' sozinho
                        // zera os integrais ao (re)ligar pra nao aplicar windup velho
                        g.controladorX.resetIntegral(); g.controladorY.resetIntegral();
                        if (!pidX && !pidY) g.driver.neutro();
                        printf("PID -> eixo X:%s  eixo Y:%s\n", pidX ? "ON" : "off", pidY ? "ON" : "off");
                    } else if (cmd == 'z' || cmd == 'Z') {
                        char sub = (linepos > 1) ? linebuf[1] : '\0';
                        float val = (linepos > 2) ? (float)atof(linebuf + 2) : 0.0f;
                        if      (sub == 'x' || sub == 'X') g.driver.ajustaNeutroX(val);
                        else if (sub == 'y' || sub == 'Y') g.driver.ajustaNeutroY(val);
                        else if (sub == 's' || sub == 'S') g.driver.salvaNeutro();
                        else if (sub == '0') g.driver.resetNeutro();
                        else printf("Comando z invalido. Use zx<n>, zy<n>, zs, z0\n");
                    } else if (cmd == 'u' || cmd == 'U') {
                        char sub = (linepos > 1) ? linebuf[1] : '\0';
                        float val = (linepos > 2) ? (float)atof(linebuf + 2) : 0.0f;
                        if      (sub == 'x' || sub == 'X') g.visao.ajustaCentroPx(val, 0.0f);
                        else if (sub == 'y' || sub == 'Y') g.visao.ajustaCentroPx(0.0f, val);
                        else if (sub == 's' || sub == 'S') g.visao.salvaCentroPx();
                        else if (sub == '0') g.visao.resetCentroPx();
                        else printf("Comando u invalido. Use ux<n>, uy<n>, us, u0\n");
                    } else if (cmd == '0') {
                        char sub = (linepos > 1) ? linebuf[1] : '\0';
                        if (sub == 'x' || sub == 'X') {
                            float val = (linepos > 2) ? (float)atof(linebuf + 2) : 0.0f;
                            g.driver.ajustaNeutroX(val);
                        } else if (sub == 'y' || sub == 'Y') {
                            float val = (linepos > 2) ? (float)atof(linebuf + 2) : 0.0f;
                            g.driver.ajustaNeutroY(val);
                        } else if (sub == 'c' || sub == 'C') {
                            char eixo = (linepos > 2) ? linebuf[2] : '\0';
                            float val = (linepos > 3) ? (float)atof(linebuf + 3) * 0.1f : 0.0f;
                            if      (eixo == 'x' || eixo == 'X') g.visao.ajustaCentroCm(val, 0.0f);
                            else if (eixo == 'y' || eixo == 'Y') g.visao.ajustaCentroCm(0.0f, val);
                            else if (eixo == 's' || eixo == 'S') g.visao.salvaCentroPx();
                            else if (eixo == '0') g.visao.resetCentroPx();
                            else printf("Comando 0c invalido. Use 0cx<n>, 0cy<n>, 0cs, 0c0\n");
                        } else if (sub == 's' || sub == 'S') {
                            g.driver.salvaNeutro();
                            g.visao.salvaCentroPx();
                        } else if (sub == '0') {
                            g.driver.resetNeutro();
                            g.visao.resetCentroPx();
                        } else {
                            printf("Comando 0 invalido. Use 0x<n>, 0y<n>, 0cx<n>, 0cy<n>, 0s, 00\n");
                        }
                    } else {
                        float val = (linepos > 1) ? (float)atof(linebuf + 1) : 0.0f;
                        pidX = pidY = false; dancaX = dancaY = false; manualAtivo = true;
                        if (cmd == 'x' || cmd == 'X') { manualX = val; printf("Servo X manual sinal=%.1f (neutro=%.1f %s)\n", manualX, g.driver.neutroX(), SERVO_X_INVERTIDO ? "inv" : ""); }
                        else                          { manualY = val; printf("Servo Y manual sinal=%.1f (neutro=%.1f)\n", manualY, g.driver.neutroY()); }
                    }
                    capturando = false; linepos = 0;
                } else if (linepos < 15) {
                    linebuf[linepos++] = (char)c;
                }
                continue;
            }
            if (c == 'x' || c == 'X' || c == 'y' || c == 'Y' || c == 'o' || c == 'O' ||
                c == 'z' || c == 'Z' || c == 'u' || c == 'U' || c == '0') {
                capturando = true; linepos = 0; linebuf[linepos++] = (char)c; continue;
            }
            // --- danca (gangorra) nao-bloqueante e neutro ---
            if (c == 'w' || c == 'W') {
                dancaX = !dancaX; pidX = pidY = false; manualAtivo = false;
                if (!dancaX && !dancaY) { danceAmp = 0.0f; g.driver.neutro(); }
                printf("Danca eixo X %s\n", dancaX ? "ON" : "OFF"); continue;
            }
            if (c == 'b' || c == 'B') {
                dancaY = !dancaY; pidX = pidY = false; manualAtivo = false;
                if (!dancaX && !dancaY) { danceAmp = 0.0f; g.driver.neutro(); }
                printf("Danca eixo Y %s\n", dancaY ? "ON" : "OFF"); continue;
            }
            if (c == 'n' || c == 'N') {
                manualAtivo = false; dancaX = dancaY = false; pidX = pidY = false;
                danceAmp = 0.0f; g.driver.neutro();
                printf("Neutro (parou manual/danca/PID)\n"); continue;
            }
            if (c == 'f' || c == 'F') { printf("Frame anotado solicitado.\n"); g.visao.solicitaDebug(); }
            else if (c == 'g') { g.visao.capturaFundo(); }
            else if (c == 'G') { g.visao.limpaFundo(); }
            else if (c == 'c' || c == 'C') { g.visao.calibraCamera(); }
            else if (c == 'm' || c == 'M') { printf("Calibrando geometria da mesa...\n"); g.visao.calibraMesaGeometrica(); }
            else if (c == 'e' || c == 'E') { sobel = !sobel; g.visao.debugSobel(sobel); printf("Sobel %s\n", sobel ? "ON" : "OFF"); }
            else if (c == 'h' || c == 'H') { mascara = !mascara; g.visao.debugMascara(mascara); printf("Mascara %s\n", mascara ? "ON" : "OFF"); }
            else if (c == 'r' || c == 'R') { g.visao.resetFiltro(); }
            else if (c == 't') { telemetria = !telemetria; printf("Telemetria %s\n", telemetria ? "ON" : "OFF"); }
            else if (c == 'T') { bool sw = g.driver.trocaEixos(); printf("Eixos dos servos: %s\n", sw ? "TROCADOS" : "normais"); }
            else if (c == '+') { g.visao.ajustaExposicao(+CAM_AEC_STEP); }
            else if (c == '-') { g.visao.ajustaExposicao(-CAM_AEC_STEP); }
            else if (c == '.') { g.visao.ajustaGanho(+1); }
            else if (c == ',') { g.visao.ajustaGanho(-1); }
            else if (c == 'a' || c == 'A') { autoSens = !autoSens; g.visao.autoSensor(autoSens); }
            else if (c == 'k' || c == 'K') { g.visao.ajustaClock(-1); }
            else if (c == 'l' || c == 'L') { g.visao.ajustaClock(+1); }
            else if (c == 'q' || c == 'Q') {
                g.visao.imprimeSensor();
                g.visao.imprimeCentroPx();
                printf("Neutro servos: X=%.1f Y=%.1f\n", g.driver.neutroX(), g.driver.neutroY());
                printf("Ganhos atuais PID: Kp=%.1f Ki=%.2f Kd=%.2f\n", g.controladorX.kp, g.controladorX.ki, g.controladorX.kd);
            }
            else if (c == 'I') {
                g.controladorX.ki += 0.1f; g.controladorY.ki += 0.1f;
                printf("Ganhos: Kp=%.1f Ki=%.2f Kd=%.2f\n", g.controladorX.kp, g.controladorX.ki, g.controladorX.kd);
            }
            else if (c == 'i') {
                g.controladorX.ki -= 0.1f; g.controladorY.ki -= 0.1f;
                if (g.controladorX.ki < 0.0f) g.controladorX.ki = 0.0f;
                if (g.controladorY.ki < 0.0f) g.controladorY.ki = 0.0f;
                printf("Ganhos: Kp=%.1f Ki=%.2f Kd=%.2f\n", g.controladorX.kp, g.controladorX.ki, g.controladorX.kd);
            }
            else if (c == 'j' || c == 'J') {
                g.driver.desconecta();                 // solta o PCA p/ a camera recriar o bus
                bool jp = g.visao.alternaCaptura();
                g.driver.iniciaMotores();              // rependura o PCA no bus novo
                printf("Captura -> %s\n", jp ? "JPEG" : "GRAYSCALE");
            }
            else if (c == 'S') { printf("Iniciando sweep automatico...\n"); sweep_sensor(g.visao); t0 = esp_timer_get_time(); frames = 0; achou_frames = 0; }
            else if (c == 'p') {
                g.controladorX.kp -= 1.0f; g.controladorY.kp -= 1.0f;
                if (g.controladorX.kp < 0.0f) g.controladorX.kp = 0.0f;
                if (g.controladorY.kp < 0.0f) g.controladorY.kp = 0.0f;
                printf("Ganhos: Kp=%.1f Ki=%.2f Kd=%.2f\n", g.controladorX.kp, g.controladorX.ki, g.controladorX.kd);
            }
            else if (c == 'P') {
                g.controladorX.kp += 1.0f; g.controladorY.kp += 1.0f;
                printf("Ganhos: Kp=%.1f Ki=%.2f Kd=%.2f\n", g.controladorX.kp, g.controladorX.ki, g.controladorX.kd);
            }
            else if (c == 'd') {
                g.controladorX.kd -= 0.1f; g.controladorY.kd -= 0.1f;
                if (g.controladorX.kd < 0.0f) g.controladorX.kd = 0.0f;
                if (g.controladorY.kd < 0.0f) g.controladorY.kd = 0.0f;
                printf("Ganhos: Kp=%.1f Ki=%.2f Kd=%.2f\n", g.controladorX.kp, g.controladorX.ki, g.controladorX.kd);
            }
            else if (c == 'D') {
                g.controladorX.kd += 0.1f; g.controladorY.kd += 0.1f;
                printf("Ganhos: Kp=%.1f Ki=%.2f Kd=%.2f\n", g.controladorX.kp, g.controladorX.ki, g.controladorX.kd);
            }
            else if (c == 'v' || c == 'V') {
                g.driver.debug();
            }
            else if (c == 's') {
                printf("Iniciando varredura rapida dos servos...\n");
                const float seq[] = { -10, 0, 10, 0 };
                for (int i = 0; i < 4; i++) {
                    printf("Servos -> %.1f graus\n", seq[i]);
                    g.driver.enviaCorrecaoX(seq[i]);
                    g.driver.enviaCorrecaoY(seq[i]);
                    vTaskDelay(pdMS_TO_TICKS(1000));
                }
                printf("Varredura concluida. Neutro.\n");
            }
            else if (c == '?') { ajuda(); }
        }

        int64_t agora = esp_timer_get_time();
        if (agora - t0 >= 5000000) {                       // a cada 5 segundos
            fps_atual = frames * 1000000.0f / (float)(agora - t0);
            g_pos_web.fps = fps_atual;
            if (!telemetria) {
                const char* pidStat = (pidX && pidY) ? "PID XY" : pidX ? "PID X" : pidY ? "PID Y" : "DESLIGADO";
                printf("[5s] fps_med=%.1f (%s) | bola: %s cm=(%.2f,%.2f) vel=(%.1f,%.1f) | achou=%d/%d atuou=%d cap=%.1fms proc=%.1fms%s%s | PID Kp=%.1f Ki=%.2f Kd=%.2f [%s]\n",
                       fps_atual, g.visao.modoJpeg() ? "JPEG" : "GRAY",
                       m.achou ? "OK" : "--", m.x, m.y, m.vx, m.vy,
                       achou_frames, frames, atuou, d.captura_us / 1000.0f, d.processo_us / 1000.0f,
                       d.overflow ? " OVERFLOW" : "", d.usou_fundo ? " [fundo]" : "",
                       g.controladorX.kp, g.controladorX.ki, g.controladorX.kd,
                       pidStat);
            }
            frames = 0;
            achou_frames = 0;
            atuou = 0;
            t0 = agora;
        }

        vTaskDelay(1);   // deixa o IDLE do core 1 respirar (watchdog)
    }
}

extern "C" void app_main(void) {
    ESP_LOGI(TAG, "Mesa Balanceadora - ETAPA 4 + CONTROLE (modo normal)");

    g.inicia();
    wifi_controle_inicia();   // AP "MesaPID" + HTTP server (core 0)

    // Camera + PID + serial no core 1, prioridade alta.
    xTaskCreatePinnedToCore(tarefa_principal, "principal", 12288, NULL, 6, NULL, 1);
}

// ============================================================
//  ETAPAS 3+ / 5 - estrutura completa com FreeRTOS dual-core
// ============================================================
#else

#include "gerenciador.h"
#include <string.h>

static ModuloGerenciador g;
static QueueHandle_t      filaMedicao;

// Flags de solicitação da serial (comunicação segura com tarefaVisao)
static volatile bool calibraMesaSolicitado = false;
static volatile bool capturaFundoSolicitado = false;
static volatile bool limpaFundoSolicitado = false;
static volatile bool alternaCapturaSolicitado = false;
static volatile int8_t adjustAecSolicitado = 0;
static volatile int8_t adjustAgcSolicitado = 0;
static volatile bool toggleAutoSensSolicitado = false;
static volatile bool telemetria = false;
static volatile bool sobel = false;

static void tarefaVisao(void* arg) {
    int frames = 0;
    int achou_frames = 0;
    int64_t t0 = esp_timer_get_time();
    float fps_atual = 0.0f;

    for (;;) {
        // Trata solicitações assíncronas do terminal de forma segura na thread da câmera
        if (calibraMesaSolicitado) {
            g.visao.calibraMesaGeometrica();
            calibraMesaSolicitado = false;
        }
        if (capturaFundoSolicitado) {
            g.visao.capturaFundo();
            capturaFundoSolicitado = false;
        }
        if (limpaFundoSolicitado) {
            g.visao.limpaFundo();
            limpaFundoSolicitado = false;
        }
        if (alternaCapturaSolicitado) {
            bool jp = g.visao.alternaCaptura();
            printf("Captura -> %s\n", jp ? "JPEG" : "GRAYSCALE");
            alternaCapturaSolicitado = false;
        }
        if (adjustAecSolicitado != 0) {
            g.visao.ajustaExposicao(adjustAecSolicitado * CAM_AEC_STEP);
            adjustAecSolicitado = 0;
        }
        if (adjustAgcSolicitado != 0) {
            g.visao.ajustaGanho(adjustAgcSolicitado * 1);
            adjustAgcSolicitado = 0;
        }
        if (toggleAutoSensSolicitado) {
            static bool autoSens = false;
            autoSens = !autoSens;
            g.visao.autoSensor(autoSens);
            toggleAutoSensSolicitado = false;
        }

        Medicao m = g.visao.detecta();
        const VisaoDebugInfo& d = g.visao.debugInfo();
        
        frames++;
        if (m.achou) achou_frames++;

        if (telemetria) {
            printf("CSV,%lld,%.3f,%.3f,%.3f,%.3f,%d,%.1f\n",
                   (long long)m.t_us, m.x, m.y, m.vx, m.vy, m.achou ? 1 : 0, fps_atual);
        }

        int64_t agora = esp_timer_get_time();
        if (agora - t0 >= 5000000) {                       // a cada 5 segundos
            fps_atual = frames * 1000000.0f / (float)(agora - t0);
            if (!telemetria) {
                printf("[5s] fps_med=%.1f (%s) | bola: %s cm=(%.2f,%.2f) vel=(%.1f,%.1f) | achou=%d/%d cap=%.1fms proc=%.1fms%s%s | PID Kp=%.1f Kd=%.2f\n",
                       fps_atual, g.visao.modoJpeg() ? "JPEG" : "GRAY",
                       m.achou ? "OK" : "--", m.x, m.y, m.vx, m.vy,
                       achou_frames, frames, d.captura_us / 1000.0f, d.processo_us / 1000.0f,
                       d.overflow ? " OVERFLOW" : "", d.usou_fundo ? " [fundo]" : "",
                       g.controladorX.kp, g.controladorX.kd);
            }
            frames = 0;
            achou_frames = 0;
            t0 = agora;
        }

        xQueueOverwrite(filaMedicao, &m);
        vTaskDelay(pdMS_TO_TICKS(5)); // evita sufocar a CPU
    }
}

static void tarefaControle(void* arg) {
    Medicao m;
    for (;;) {
        // Timeout pequeno de 2ms para que processaFila() seja chamado com alta frequência e baixo jitter
        if (xQueueReceive(filaMedicao, &m, pdMS_TO_TICKS(2)) == pdTRUE) {
            g.calculaAcaoControle(m);
        }
        g.processaFila();
    }
}

static void tarefaSerial(void* arg) {
    // Menu inicial de comandos
    printf("\n========================================================\n");
    printf(" MESA BALANCEADORA PID - ETAPA 5 (MALHA FECHADA ATIVA)\n");
    printf("========================================================\n");
    printf(" Comandos do Sensor/Mesa:\n");
    printf("   f : Capturar frame base64 com overlays\n");
    printf("   g : Capturar fundo da mesa vazia (subtracao ativa)\n");
    printf("   G : Descartar fundo (usa grade de iluminacao)\n");
    printf("   m : Calibrar geometria e homografia da mesa\n");
    printf("   e : Ligar/desligar overlay do filtro Sobel\n");
    printf("   t : Ligar/desligar telemetria CSV\n");
    printf("   j : Alternar modo do sensor (JPEG <-> GRAYSCALE)\n");
    printf("   + / - : Incrementar/Decrementar exposicao (AEC)\n");
    printf("   . / , : Incrementar/Decrementar ganho (AGC)\n");
    printf("   a : Ligar/desligar modo automatico do sensor\n");
    printf("   i : Mostrar info do sensor e ganhos do PID\n");
    printf("   r : Resetar filtro alfa-beta\n");
    printf(" Comandos do PID:\n");
    printf("   P / p : Aumentar/Diminuir Proporcional (Kp) em 10\n");
    printf("   D / d : Aumentar/Diminuir Derivativo (Kd) em 0.1\n");
    printf("========================================================\n\n");

    for (;;) {
        int c = getchar();
        if (c >= 0) {
            if (c == 'f' || c == 'F') {
                printf("Frame anotado solicitado.\n");
                g.visao.solicitaDebug();
            }
            else if (c == 'g') {
                capturaFundoSolicitado = true;
            }
            else if (c == 'G') {
                limpaFundoSolicitado = true;
            }
            else if (c == 'm' || c == 'M') {
                printf("Calibrando geometria da mesa...\n");
                calibraMesaSolicitado = true;
            }
            else if (c == 'e' || c == 'E') {
                sobel = !sobel;
                g.visao.debugSobel(sobel);
                printf("Sobel %s\n", sobel ? "ON" : "OFF");
            }
            else if (c == 'r' || c == 'R') {
                g.visao.resetFiltro();
            }
            else if (c == 't' || c == 'T') {
                telemetria = !telemetria;
                printf("Telemetria %s\n", telemetria ? "ON" : "OFF");
            }
            else if (c == '+') {
                adjustAecSolicitado = +1;
            }
            else if (c == '-') {
                adjustAecSolicitado = -1;
            }
            else if (c == '.') {
                adjustAgcSolicitado = +1;
            }
            else if (c == ',') {
                adjustAgcSolicitado = -1;
            }
            else if (c == 'a' || c == 'A') {
                toggleAutoSensSolicitado = true;
            }
            else if (c == 'j' || c == 'J') {
                alternaCapturaSolicitado = true;
            }
            else if (c == 'p') {
                g.controladorX.kp -= 10.0f;
                g.controladorY.kp -= 10.0f;
                if (g.controladorX.kp < 0.0f) g.controladorX.kp = 0.0f;
                if (g.controladorY.kp < 0.0f) g.controladorY.kp = 0.0f;
                printf("Ganhos: Kp=%.1f Ki=%.2f Kd=%.2f\n", g.controladorX.kp, g.controladorX.ki, g.controladorX.kd);
            }
            else if (c == 'P') {
                g.controladorX.kp += 10.0f;
                g.controladorY.kp += 10.0f;
                printf("Ganhos: Kp=%.1f Ki=%.2f Kd=%.2f\n", g.controladorX.kp, g.controladorX.ki, g.controladorX.kd);
            }
            else if (c == 'd') {
                g.controladorX.kd -= 0.1f;
                g.controladorY.kd -= 0.1f;
                if (g.controladorX.kd < 0.0f) g.controladorX.kd = 0.0f;
                if (g.controladorY.kd < 0.0f) g.controladorY.kd = 0.0f;
                printf("Ganhos: Kp=%.1f Ki=%.2f Kd=%.2f\n", g.controladorX.kp, g.controladorX.ki, g.controladorX.kd);
            }
            else if (c == 'D') {
                g.controladorX.kd += 0.1f;
                g.controladorY.kd += 0.1f;
                printf("Ganhos: Kp=%.1f Ki=%.2f Kd=%.2f\n", g.controladorX.kp, g.controladorX.ki, g.controladorX.kd);
            }
            else if (c == 'i' || c == 'I') {
                g.visao.imprimeSensor();
                printf("Ganhos atuais: Kp=%.1f  Kd=%.2f\n", g.controladorX.kp, g.controladorX.kd);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

extern "C" void app_main(void) {
    ESP_LOGI(TAG, "Mesa Balanceadora - ETAPA %d", ETAPA);
    iniciaMaquinaEstados();
    g.inicia();
    filaMedicao = xQueueCreate(1, sizeof(Medicao));
    xTaskCreatePinnedToCore(tarefaVisao,    "visao",    8192, NULL, PRIO_VISAO,    NULL, NUCLEO_VISAO);
    xTaskCreatePinnedToCore(tarefaControle, "controle", 4096, NULL, PRIO_CONTROLE, NULL, NUCLEO_CONTROLE);
    xTaskCreatePinnedToCore(tarefaSerial,   "serial",   4096, NULL, PRIO_COMMS,    NULL, NUCLEO_COMMS);
}

#endif
