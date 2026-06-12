#include "visao.h"
#include "config.h"
#include "esp_camera.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char* TAG = "VISAO";

bool Visao::inicia() {
    camera_config_t cfg = {};

    // Pinos do hardware (AI-Thinker ESP32-CAM / WROVER)
    cfg.pin_pwdn     = CAM_PIN_PWDN;
    cfg.pin_reset    = CAM_PIN_RESET;
    cfg.pin_xclk     = CAM_PIN_XCLK;
    cfg.pin_sccb_sda = CAM_PIN_SIOD;
    cfg.pin_sccb_scl = CAM_PIN_SIOC;
    cfg.pin_d7       = CAM_PIN_D7;
    cfg.pin_d6       = CAM_PIN_D6;
    cfg.pin_d5       = CAM_PIN_D5;
    cfg.pin_d4       = CAM_PIN_D4;
    cfg.pin_d3       = CAM_PIN_D3;
    cfg.pin_d2       = CAM_PIN_D2;
    cfg.pin_d1       = CAM_PIN_D1;
    cfg.pin_d0       = CAM_PIN_D0;
    cfg.pin_vsync    = CAM_PIN_VSYNC;
    cfg.pin_href     = CAM_PIN_HREF;
    cfg.pin_pclk     = CAM_PIN_PCLK;

    cfg.xclk_freq_hz = CAM_XCLK_HZ;
    cfg.ledc_timer   = LEDC_TIMER_0;
    cfg.ledc_channel = LEDC_CHANNEL_0;
    cfg.fb_count     = CAM_FB_COUNT;

#if ETAPA == 1
    // Etapa 1: JPEG para streaming facil pelo navegador
    cfg.pixel_format = PIXFORMAT_JPEG;
    cfg.frame_size   = CAM_STREAM_FRAMESIZE;   // QVGA 320x240
    cfg.jpeg_quality = CAM_JPEG_QUALITY;
    cfg.fb_location  = CAMERA_FB_IN_PSRAM;     // WROVER tem PSRAM
    cfg.grab_mode    = CAMERA_GRAB_LATEST;     // sempre o frame mais recente
#else
    // Etapa 4+: RGB565 para leitura pixel a pixel
    cfg.pixel_format = PIXFORMAT_RGB565;
    cfg.frame_size   = CAM_DETECT_FRAMESIZE;   // QQVGA 160x120
    cfg.jpeg_quality = 0;                      // nao usado em RGB565
    cfg.fb_location  = CAMERA_FB_IN_PSRAM;
    cfg.grab_mode    = CAMERA_GRAB_LATEST;
#endif

    esp_err_t err = esp_camera_init(&cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_camera_init falhou: 0x%x", err);
        ESP_LOGE(TAG, "Verifique os pinos e se o cabo flat esta bem encaixado.");
        return false;
    }

    // Ajustes do sensor OV2640 (opcionais, mude se a imagem ficar ruim)
    sensor_t* s = esp_camera_sensor_get();
    s->set_brightness(s, 0);     // -2 a 2
    s->set_contrast(s, 0);       // -2 a 2
    s->set_saturation(s, 0);
    s->set_whitebal(s, 1);       // auto white balance ligado
    s->set_exposure_ctrl(s, 1);  // auto exposicao ligada

    ESP_LOGI(TAG, "Camera OV2640 iniciada com sucesso");
    return true;
}

Medicao Visao::detecta() {
    // TODO (Etapa 4): varrer pixels RGB565, threshold de cor, centroide
    Medicao m;
    m.x     = 0.0f;
    m.y     = 0.0f;
    m.achou = false;
    m.t_us  = esp_timer_get_time();
    return m;
}

void Visao::calibraCor() {
    // TODO (proximos passos): auto-calibracao pela NVS
}
