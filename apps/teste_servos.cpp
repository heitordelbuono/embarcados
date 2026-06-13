// ============================================================
//  apps/teste_servos.cpp  (era ETAPA 2)
//  Teste interativo dos servos via PCA9685 (DriverAtuacao).
//  Acha o neutro e os limites mecanicos; comandos pela serial.
//
//  Para compilar: troque "main.cpp" por "../apps/teste_servos.cpp"
//  no SRCS de src/CMakeLists.txt (ver apps/README.md).
// ============================================================
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "config.h"
#include "driver_atuacao.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <fcntl.h>

static const char* TAG = "MESA";
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
    ESP_LOGI(TAG, "Mesa Balanceadora - teste de servos (interativo)");
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
