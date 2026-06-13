// ============================================================
//  apps/teste_camera.cpp  (era ETAPA 1)
//  So a camera: inicializa a OV2640 em VGA JPEG e:
//    MODO_CAMERA 0 -> streaming MJPEG pelo WiFi (abre no navegador)
//    MODO_CAMERA 1 -> teste SEM WiFi: foto pela serial + FPS de captura
//
//  Faz a propria init da camera (VGA JPEG) para nao depender do modulo
//  de visao de producao, que e QQVGA voltado a deteccao.
//
//  Para compilar: troque "main.cpp" por "../apps/teste_camera.cpp"
//  no SRCS de src/CMakeLists.txt e inclua tambem wifi_stream.cpp /
//  camera_teste.cpp conforme o MODO_CAMERA (ver apps/README.md).
// ============================================================
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_camera.h"
#include "config.h"
#include "wifi_stream.h"
#include "camera_teste.h"

static const char* TAG = "MESA";

static esp_err_t inicia_camera_vga_jpeg() {
    camera_config_t cfg = {};
    cfg.pin_pwdn = CAM_PIN_PWDN; cfg.pin_reset = CAM_PIN_RESET; cfg.pin_xclk = CAM_PIN_XCLK;
    cfg.pin_sccb_sda = CAM_PIN_SIOD; cfg.pin_sccb_scl = CAM_PIN_SIOC;
    cfg.pin_d7 = CAM_PIN_D7; cfg.pin_d6 = CAM_PIN_D6; cfg.pin_d5 = CAM_PIN_D5; cfg.pin_d4 = CAM_PIN_D4;
    cfg.pin_d3 = CAM_PIN_D3; cfg.pin_d2 = CAM_PIN_D2; cfg.pin_d1 = CAM_PIN_D1; cfg.pin_d0 = CAM_PIN_D0;
    cfg.pin_vsync = CAM_PIN_VSYNC; cfg.pin_href = CAM_PIN_HREF; cfg.pin_pclk = CAM_PIN_PCLK;
    cfg.xclk_freq_hz = CAM_XCLK_HZ;
    cfg.ledc_timer = LEDC_TIMER_0; cfg.ledc_channel = LEDC_CHANNEL_0;
    cfg.fb_count = CAM_FB_COUNT;
    cfg.fb_location = CAMERA_FB_IN_PSRAM;
    cfg.grab_mode = CAMERA_GRAB_LATEST;
    cfg.pixel_format = PIXFORMAT_JPEG;
    cfg.frame_size   = CAM_STREAM_FRAMESIZE;
    cfg.jpeg_quality = CAM_JPEG_QUALITY;
    return esp_camera_init(&cfg);
}

extern "C" void app_main(void) {
    ESP_LOGI(TAG, "Mesa Balanceadora - teste de camera");

    if (inicia_camera_vga_jpeg() != ESP_OK) {
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
