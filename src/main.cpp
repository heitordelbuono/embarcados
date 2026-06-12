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
#include "esp_random.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <fcntl.h>

static DriverAtuacao driver;

static void ajuda() {
    printf("\n=== CONTROLE DE SERVOS ===\n");
    printf("  x<sinal>  -> servo X, -%d..+%d graus  (ex: x-10  x15)\n", SERVO_RANGE, SERVO_RANGE);
    printf("  y<sinal>  -> servo Y, -%d..+%d graus  (ex: y-10  y15)\n", SERVO_RANGE, SERVO_RANGE);
    printf("  p<canal>,<us> -> pulso direto em us  (ex: p0,1500)\n");
    printf("  n         -> neutro (mesa plana)\n");
    printf("  s         -> varredura: -%d -> 0 -> +%d -> 0\n", SERVO_RANGE, SERVO_RANGE);
    printf("  a         -> modo danca (aleatorio, Enter para parar)\n");
    printf("  d         -> debug: I2C scan + registradores PCA9685\n");
    printf("  ?         -> esta ajuda\n");
    printf("  Neutros: X=%d  Y=%d  |  Range: +/-%d  |  X %s\n",
           SERVO_NEUTRO_X, SERVO_NEUTRO_Y, SERVO_RANGE,
           SERVO_X_INVERTIDO ? "(invertido)" : "(normal)");
    printf("==========================\n\n");
}

// Retorna float aleatorio em [-range, +range]
static float rand_range(float range) {
    // esp_random() retorna uint32_t; mapeia para [-1.0, +1.0] * range
    return ((float)(int32_t)esp_random() / (float)INT32_MAX) * range;
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
            } else if (cmd == 'a' || cmd == 'A') {
                printf("Modo danca suave (+-%.0f graus). Enter para parar.\n",
                       (float)SERVO_DANCE_RANGE);

                // stdin nao-bloqueante so durante a danca
                int sflags = fcntl(fileno(stdin), F_GETFL, 0);
                fcntl(fileno(stdin), F_SETFL, sflags | O_NONBLOCK);
                // drena o \n do proprio comando 'a' para nao parar imediatamente
                while (getchar() != EOF) {}

                float cx = 0, cy = 0;
                float tx = rand_range(SERVO_DANCE_RANGE);
                float ty = rand_range(SERVO_DANCE_RANGE);
                int hold = 0;
                bool parar = false;
                while (!parar) {
                    float dx = tx - cx, dy = ty - cy;
                    cx += (fabsf(dx) > SERVO_DANCE_SPEED) ? (dx > 0 ? SERVO_DANCE_SPEED : -SERVO_DANCE_SPEED) : dx;
                    cy += (fabsf(dy) > SERVO_DANCE_SPEED) ? (dy > 0 ? SERVO_DANCE_SPEED : -SERVO_DANCE_SPEED) : dy;
                    driver.enviaCorrecaoX(cx);
                    driver.enviaCorrecaoY(cy);

                    if (fabsf(cx - tx) < 0.1f && fabsf(cy - ty) < 0.1f) {
                        if (++hold >= (SERVO_DANCE_HOLD_MS / 20)) {
                            tx = rand_range(SERVO_DANCE_RANGE);
                            ty = rand_range(SERVO_DANCE_RANGE);
                            printf("  alvo X=%.1f  Y=%.1f\n", tx, ty);
                            hold = 0;
                        }
                    }

                    vTaskDelay(pdMS_TO_TICKS(20));
                    int c2 = getchar();  // retorna -1 imediatamente se nao ha tecla
                    if (c2 == '\n' || c2 == '\r') parar = true;
                }

                fcntl(fileno(stdin), F_SETFL, sflags);  // restaura stdin bloqueante
                driver.neutro();
                printf("Danca parada. Neutro.\n");
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
//  ETAPAS 2+ - estrutura completa com FreeRTOS dual-core
// ============================================================
#else

#include "gerenciador.h"

static ModuloGerenciador g;
static QueueHandle_t      filaMedicao;

static void tarefaVisao(void* arg) {
    for (;;) {
        Medicao m = g.visao.detecta();
        xQueueOverwrite(filaMedicao, &m);
        // vTaskDelay so se precisar limitar FPS; por ora roda livre
    }
}

static void tarefaControle(void* arg) {
    Medicao m;
    for (;;) {
        if (xQueueReceive(filaMedicao, &m, pdMS_TO_TICKS(50)) == pdTRUE)
            g.calculaAcaoControle(m);
        g.processaFila();
    }
}

static void tarefaComms(void* arg) {
    for (;;) {
        // TODO (proximos passos): BLE - botao, setpoint, ganhos, telemetria
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

extern "C" void app_main(void) {
    ESP_LOGI(TAG, "Mesa Balanceadora - ETAPA %d", ETAPA);
    iniciaMaquinaEstados();
    g.inicia();
    filaMedicao = xQueueCreate(1, sizeof(Medicao));
    xTaskCreatePinnedToCore(tarefaVisao,    "visao",    8192, NULL, PRIO_VISAO,    NULL, NUCLEO_VISAO);
    xTaskCreatePinnedToCore(tarefaControle, "controle", 4096, NULL, PRIO_CONTROLE, NULL, NUCLEO_CONTROLE);
    xTaskCreatePinnedToCore(tarefaComms,    "comms",    4096, NULL, PRIO_COMMS,    NULL, NUCLEO_COMMS);
}

#endif
