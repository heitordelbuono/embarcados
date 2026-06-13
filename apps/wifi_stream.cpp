// ============================================================
//  wifi_stream.cpp - Etapa 1
//  Conecta ao WiFi e serve stream MJPEG na porta HTTP_PORT.
//  Abra  http://<IP_DA_PLACA>/stream  no navegador.
//  O FPS e impresso na serial a cada segundo.
// ============================================================
#include "wifi_stream.h"
#include "config.h"
#include "esp_camera.h"
#include "esp_http_server.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include <string.h>
#include <stdio.h>

static const char* TAG = "WIFI";

#define WIFI_CONECTADO_BIT BIT0
#define WIFI_FALHOU_BIT    BIT1
static EventGroupHandle_t wifi_events;

// ---- Contador de FPS (roda numa tarefa separada) ---------
static int fps_counter = 0;
static uint64_t captura_soma_us = 0;
static uint64_t captura_max_us = 0;
static uint64_t envio_soma_us = 0;
static uint64_t envio_max_us = 0;
static uint64_t bytes_soma = 0;

static void tarefa_fps(void* arg) {
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        int frames = fps_counter;
        uint32_t cap_avg_ms = frames ? (uint32_t)(captura_soma_us / frames / 1000) : 0;
        uint32_t cap_max_ms = (uint32_t)(captura_max_us / 1000);
        uint32_t send_avg_ms = frames ? (uint32_t)(envio_soma_us / frames / 1000) : 0;
        uint32_t send_max_ms = (uint32_t)(envio_max_us / 1000);
        uint32_t bytes_avg = frames ? (uint32_t)(bytes_soma / frames) : 0;

        ESP_LOGI("FPS", "%d fps | captura avg/max=%u/%u ms | envio avg/max=%u/%u ms | frame medio=%u B",
                 frames, cap_avg_ms, cap_max_ms, send_avg_ms, send_max_ms, bytes_avg);

        fps_counter = 0;
        captura_soma_us = 0;
        captura_max_us = 0;
        envio_soma_us = 0;
        envio_max_us = 0;
        bytes_soma = 0;
    }
}

// ---- Handler do stream MJPEG ----------------------------
#define BOUNDARY "mjpegboundary"
static const char* CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" BOUNDARY;
static const char* FRAME_HDR    = "--" BOUNDARY "\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";
static const char* FRAME_TAIL   = "\r\n";

static esp_err_t handler_stream(httpd_req_t* req) {
    esp_err_t res = httpd_resp_set_type(req, CONTENT_TYPE);
    if (res != ESP_OK) return res;
    // desabilita timeout para stream continuo
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    char hdr_buf[96];   // header MJPEG cabe em ~74 bytes
    for (;;) {
        int64_t t0 = esp_timer_get_time();
        camera_fb_t* fb = esp_camera_fb_get();
        int64_t t1 = esp_timer_get_time();
        if (!fb) { ESP_LOGE(TAG, "falha ao capturar frame"); break; }
        size_t frame_len = fb->len;

        int hdr_len = snprintf(hdr_buf, sizeof(hdr_buf), FRAME_HDR, frame_len);
        res  = httpd_resp_send_chunk(req, hdr_buf, hdr_len);
        if (res == ESP_OK) res = httpd_resp_send_chunk(req, (char*)fb->buf, frame_len);
        if (res == ESP_OK) res = httpd_resp_send_chunk(req, FRAME_TAIL, strlen(FRAME_TAIL));
        int64_t t2 = esp_timer_get_time();

        esp_camera_fb_return(fb);
        fps_counter++;

        uint64_t captura_us = (uint64_t)(t1 - t0);
        uint64_t envio_us = (uint64_t)(t2 - t1);
        captura_soma_us += captura_us;
        envio_soma_us += envio_us;
        bytes_soma += frame_len;
        if (captura_us > captura_max_us) captura_max_us = captura_us;
        if (envio_us > envio_max_us) envio_max_us = envio_us;

        if (res != ESP_OK) break;   // cliente desconectou
    }
    return ESP_OK;
}

// ---- Handler da pagina raiz (mostra IP e link) ----------
static esp_err_t handler_root(httpd_req_t* req) {
    const char* html =
        "<html><body style='font-family:sans-serif'>"
        "<h2>ESP32-CAM Mesa Balanceadora</h2>"
        "<img src='/stream' style='max-width:100%'>"
        "</body></html>";
    return httpd_resp_send(req, html, strlen(html));
}

static void inicia_servidor_http() {
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.server_port    = HTTP_PORT;
    cfg.ctrl_port      = 32768;

    httpd_handle_t server = NULL;
    if (httpd_start(&server, &cfg) != ESP_OK) {
        ESP_LOGE(TAG, "falha ao iniciar servidor HTTP");
        return;
    }

    httpd_uri_t uri_root   = { .uri = "/",       .method = HTTP_GET, .handler = handler_root,   .user_ctx = NULL };
    httpd_uri_t uri_stream = { .uri = "/stream",  .method = HTTP_GET, .handler = handler_stream, .user_ctx = NULL };
    httpd_register_uri_handler(server, &uri_root);
    httpd_register_uri_handler(server, &uri_stream);

    ESP_LOGI(TAG, "servidor HTTP em http://[IP]:%d  |  stream: /stream", HTTP_PORT);
}

// ---- Callback do WiFi -----------------------------------
static void wifi_callback(void* arg, esp_event_base_t base, int32_t id, void* data) {
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(TAG, "desconectado, reconectando...");
        esp_wifi_connect();
        xEventGroupClearBits(wifi_events, WIFI_CONECTADO_BIT);
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* ev = (ip_event_got_ip_t*)data;
        ESP_LOGI(TAG, "═══════════════════════════════════════");
        ESP_LOGI(TAG, "  WiFi conectado!");
        ESP_LOGI(TAG, "  IP: " IPSTR, IP2STR(&ev->ip_info.ip));
        ESP_LOGI(TAG, "  Abra no navegador: http://" IPSTR, IP2STR(&ev->ip_info.ip));
        ESP_LOGI(TAG, "  Stream:  http://" IPSTR "/stream", IP2STR(&ev->ip_info.ip));
        ESP_LOGI(TAG, "═══════════════════════════════════════");
        xEventGroupSetBits(wifi_events, WIFI_CONECTADO_BIT);
    }
}

// ---- Ponto de entrada publico ---------------------------
void wifi_stream_inicia() {
    // NVS (necessario para WiFi)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();

    wifi_events = xEventGroupCreate();

    wifi_init_config_t wcfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&wcfg);

    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_callback, NULL);
    esp_event_handler_register(IP_EVENT,   IP_EVENT_STA_GOT_IP, wifi_callback, NULL);

    wifi_config_t wclient = {};
    strncpy((char*)wclient.sta.ssid,     WIFI_SSID, sizeof(wclient.sta.ssid) - 1);
    strncpy((char*)wclient.sta.password, WIFI_PASS, sizeof(wclient.sta.password) - 1);

    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wclient);
    esp_wifi_start();

    // Desliga o power-save do WiFi: e o que mais derruba o FPS do streaming.
    esp_wifi_set_ps(WIFI_PS_NONE);

    ESP_LOGI(TAG, "Conectando ao WiFi '%s'...", WIFI_SSID);
    xEventGroupWaitBits(wifi_events, WIFI_CONECTADO_BIT, pdFALSE, pdTRUE, portMAX_DELAY);

    inicia_servidor_http();

    xTaskCreatePinnedToCore(tarefa_fps, "fps", 2048, NULL, 2, NULL, 0);
}
