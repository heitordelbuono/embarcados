// ============================================================
//  main.cpp - Mesa Balanceadora PID
//
//  Loop unico (camera + deteccao + PID + servos) numa task no CORE 1.
//  O WiFi/HTTP (interface no celular) rodam no CORE 0 via wifi_controle.
//  Comandos pela serial ficam em comandos_serial.cpp.
//
//  Testes de hardware (camera, servos, PCA9685, LEDC) estao em apps/.
// ============================================================
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <math.h>
#include <fcntl.h>
#include <stdio.h>

#include "config.h"
#include "tipos.h"
#include "gerenciador.h"
#include "comandos_serial.h"
#include "wifi_controle.h"

static const char* TAG = "MESA";

static ModuloGerenciador g;

// Loop principal no CORE 1, longe do WiFi (que fica no core 0). Rodar isto
// no app_main (core 0) derrubava o FPS e estourava o watchdog do IDLE0.
static void tarefa_principal(void* arg) {
    int sflags = fcntl(fileno(stdin), F_GETFL, 0);
    fcntl(fileno(stdin), F_SETFL, sflags | O_NONBLOCK);

    EstadoUI ui;
    comandos_ajuda(g);

    int     frames = 0, achou_frames = 0, atuou = 0;
    int64_t t0 = esp_timer_get_time();

    for (;;) {
        Medicao m = g.visao.detecta();
        const VisaoDebugInfo& d = g.visao.debugInfo();
        frames++;
        if (m.achou) achou_frames++;

        // Posicao e setpoint para a interface web (lidos pelo celular)
        wifi_controle_atualiza_posicao(m.x, m.y, m.achou, ui.fps_atual);
        wifi_controle_le_setpoint(&g.setpointX, &g.setpointY);

        // Prioridade de atuacao: PID > danca > manual > neutro
        if (ui.pidX || ui.pidY) {
            g.calculaAcaoControle(m, ui.pidX, ui.pidY);
            if (m.achou) atuou++;
        } else if (ui.dancaX || ui.dancaY) {
            if (ui.danceAmp < SERVO_DANCE_RANGE) {
                ui.danceAmp += 0.08f;
                if (ui.danceAmp > SERVO_DANCE_RANGE) ui.danceAmp = SERVO_DANCE_RANGE;
            }
            float sx = sinf(ui.danceFaseX) * ui.danceAmp;
            float sy = sinf(ui.danceFaseY) * ui.danceAmp;
            g.driver.enviaCorrecaoX(ui.dancaX ? sx : 0.0f);
            g.driver.enviaCorrecaoY(ui.dancaY ? sy : 0.0f);
            ui.danceFaseX += SERVO_DANCE_SPEED_X * 0.03f;
            ui.danceFaseY += SERVO_DANCE_SPEED_Y * 0.03f;
            if (ui.danceFaseX >= 2.0f * (float)M_PI) ui.danceFaseX -= 2.0f * (float)M_PI;
            if (ui.danceFaseY >= 2.0f * (float)M_PI) ui.danceFaseY -= 2.0f * (float)M_PI;
        } else if (ui.manualAtivo) {
            g.driver.enviaCorrecaoX(ui.manualX);
            g.driver.enviaCorrecaoY(ui.manualY);
        } else {
            g.driver.neutro();
        }
        g.processaFila();

        if (ui.telemetria) {
            printf("CSV,%lld,%.3f,%.3f,%.3f,%.3f,%d,%.1f\n",
                   (long long)m.t_us, m.x, m.y, m.vx, m.vy, m.achou ? 1 : 0, ui.fps_atual);
        }

        comandos_processa(g, ui);

        int64_t agora = esp_timer_get_time();
        if (agora - t0 >= 5000000) {                       // a cada 5 segundos
            ui.fps_atual = frames * 1000000.0f / (float)(agora - t0);
            if (!ui.telemetria) {
                const char* pidStat = (ui.pidX && ui.pidY) ? "PID XY" : ui.pidX ? "PID X" : ui.pidY ? "PID Y" : "DESLIGADO";
                printf("[5s] fps_med=%.1f | bola: %s cm=(%.2f,%.2f) vel=(%.1f,%.1f) | achou=%d/%d atuou=%d cap=%.1fms proc=%.1fms%s%s | PID Kp=%.1f Ki=%.2f Kd=%.2f [%s]\n",
                       ui.fps_atual,
                       m.achou ? "OK" : "--", m.x, m.y, m.vx, m.vy,
                       achou_frames, frames, atuou, d.captura_us / 1000.0f, d.processo_us / 1000.0f,
                       d.overflow ? " OVERFLOW" : "", d.usou_fundo ? " [fundo]" : "",
                       g.controladorX.kp, g.controladorX.ki, g.controladorX.kd,
                       pidStat);
            }
            frames = 0; achou_frames = 0; atuou = 0; t0 = agora;
        }

        vTaskDelay(1);   // deixa o IDLE do core 1 respirar (watchdog)
    }
}

extern "C" void app_main(void) {
    ESP_LOGI(TAG, "Mesa Balanceadora PID");

    g.inicia();
    wifi_controle_inicia();   // AP "MesaPID" + HTTP server (core 0)

    xTaskCreatePinnedToCore(tarefa_principal, "principal", 12288, NULL, 15, NULL, 1);
}
