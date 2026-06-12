// ============================================================
//  camera_teste.cpp - Etapa 1, modo SEM WiFi
//  Mede o FPS real de captura e manda 1 foto pela serial.
// ============================================================
#include "camera_teste.h"
#include "config.h"
#include "esp_camera.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <stdint.h>

static const char* TAG = "CAMTESTE";

#if !CAMERA_SWEEP_FPS
// Imprime os bytes em base64, em linhas, entre marcadores que o
// script recebe_foto.py procura. Sem dependencia externa.
static void manda_foto_base64(const uint8_t* data, size_t len) {
    static const char* T =
        "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    char line[80];
    int col = 0;
    size_t i = 0;

    printf("\n===FOTO_BASE64_INICIO===\n");
    while (i + 3 <= len) {
        uint32_t n = (data[i] << 16) | (data[i + 1] << 8) | data[i + 2];
        line[col++] = T[(n >> 18) & 63];
        line[col++] = T[(n >> 12) & 63];
        line[col++] = T[(n >> 6)  & 63];
        line[col++] = T[ n        & 63];
        i += 3;
        if (col >= 76) { line[col] = 0; printf("%s\n", line); col = 0; }
    }
    size_t rem = len - i;                  // 0, 1 ou 2 bytes finais
    if (rem == 1) {
        uint32_t n = data[i] << 16;
        line[col++] = T[(n >> 18) & 63];
        line[col++] = T[(n >> 12) & 63];
        line[col++] = '=';
        line[col++] = '=';
    } else if (rem == 2) {
        uint32_t n = (data[i] << 16) | (data[i + 1] << 8);
        line[col++] = T[(n >> 18) & 63];
        line[col++] = T[(n >> 12) & 63];
        line[col++] = T[(n >> 6)  & 63];
        line[col++] = '=';
    }
    if (col) { line[col] = 0; printf("%s\n", line); }
    printf("===FOTO_BASE64_FIM===\n");
}
#endif

void camera_teste_benchmark() {
    ESP_LOGI(TAG, "======= MODO TESTE SEM WIFI =======");
    vTaskDelay(pdMS_TO_TICKS(500));

#if CAMERA_SWEEP_FPS
    struct Caso {
        framesize_t tamanho;
        const char* nome;
    };

    const Caso casos[] = {
        {FRAMESIZE_QQVGA, "QQVGA 160x120"},
        {FRAMESIZE_HQVGA, "HQVGA 240x176"},
        {FRAMESIZE_QVGA,  "QVGA  320x240"},
        {FRAMESIZE_CIF,   "CIF   400x296"},
        {FRAMESIZE_HVGA,  "HVGA  480x320"},
        {FRAMESIZE_VGA,   "VGA   640x480"},
    };

    sensor_t* s = esp_camera_sensor_get();
    if (!s) {
        ESP_LOGE(TAG, "sensor nao disponivel");
        return;
    }

    ESP_LOGI(TAG, "Sweep local de FPS JPEG, sem WiFi, %d s por resolucao",
             CAMERA_SWEEP_SEGUNDOS);
    ESP_LOGI(TAG, "Resolucao,FPS,FrameMedioBytes,CapturaMediaMs,CapturaMaxMs");

    for (const Caso& caso : casos) {
        ESP_LOGI(TAG, "Testando %s...", caso.nome);
        if (s->set_framesize(s, caso.tamanho) != 0) {
            ESP_LOGW(TAG, "nao consegui ajustar %s", caso.nome);
            continue;
        }

        vTaskDelay(pdMS_TO_TICKS(500));
        for (int i = 0; i < 3; i++) {
            camera_fb_t* warm = esp_camera_fb_get();
            if (warm) esp_camera_fb_return(warm);
        }

        int frames = 0;
        size_t soma_bytes = 0;
        int64_t soma_captura_us = 0;
        int64_t max_captura_us = 0;
        int64_t t0 = esp_timer_get_time();
        int64_t limite = t0 + (int64_t)CAMERA_SWEEP_SEGUNDOS * 1000000LL;

        while (esp_timer_get_time() < limite) {
            int64_t tc0 = esp_timer_get_time();
            camera_fb_t* f = esp_camera_fb_get();
            int64_t captura_us = esp_timer_get_time() - tc0;
            if (!f) continue;
            frames++;
            soma_bytes += f->len;
            soma_captura_us += captura_us;
            if (captura_us > max_captura_us) max_captura_us = captura_us;
            esp_camera_fb_return(f);
        }

        int64_t t1 = esp_timer_get_time();
        float segundos = (float)(t1 - t0) / 1000000.0f;
        float fps = segundos > 0 ? (float)frames / segundos : 0.0f;
        unsigned medio = (unsigned)(soma_bytes / (frames ? frames : 1));
        float captura_media_ms = frames ? (float)soma_captura_us / (float)frames / 1000.0f : 0.0f;
        float captura_max_ms = (float)max_captura_us / 1000.0f;
        ESP_LOGI(TAG, "%s,%.1f,%u,%.1f,%.1f",
                 caso.nome, fps, medio, captura_media_ms, captura_max_ms);
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    ESP_LOGI(TAG, "Sweep concluido. Reinicie ou grave MODO_CAMERA=0 para stream WiFi.");
    for (;;) vTaskDelay(pdMS_TO_TICKS(10000));
#else
    // ---- 1) Manda uma foto pela serial ----
    camera_fb_t* fb = esp_camera_fb_get();
    if (fb) {
        ESP_LOGI(TAG, "Foto capturada: %dx%d, %u bytes (JPEG)",
                 fb->width, fb->height, (unsigned)fb->len);
        ESP_LOGI(TAG, "Mandando pela serial... (use tools/recebe_foto.py)");
        manda_foto_base64(fb->buf, fb->len);
        esp_camera_fb_return(fb);
    } else {
        ESP_LOGE(TAG, "falha ao capturar a foto");
    }
    vTaskDelay(pdMS_TO_TICKS(1000));

    // ---- 2) Benchmark de FPS de captura (sem transmitir) ----
    ESP_LOGI(TAG, "Medindo FPS real de captura local (Ctrl+C p/ sair)...");
    int     frames = 0;
    size_t  soma_bytes = 0;
    int64_t t0 = esp_timer_get_time();

    for (;;) {
        camera_fb_t* f = esp_camera_fb_get();
        if (!f) continue;
        frames++;
        soma_bytes += f->len;
        esp_camera_fb_return(f);

        int64_t agora = esp_timer_get_time();
        if (agora - t0 >= 1000000) {
            ESP_LOGI(TAG, "captura: %d fps | frame medio: %u bytes",
                     frames, (unsigned)(soma_bytes / (frames ? frames : 1)));
            frames = 0;
            soma_bytes = 0;
            t0 = agora;
        }
    }
#endif
}
