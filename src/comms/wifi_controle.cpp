// ============================================================
//  wifi_controle.cpp
//  AP "MesaPID" + HTTP server + WebSocket.
//    GET /       -> pagina HTML com canvas animado
//    GET /ws     -> WebSocket persistente:
//                     browser  -> {"cmd":"tick"}                (a cada 50ms)
//                     browser  -> {"cmd":"sp","x":1.2,"y":-0.5} (clique)
//                     ESP32    -> {"x":..,"y":..,"a":..,"f":..,"sx":..,"sy":..}
//    GET /health -> JSON de diagnostico
//
//  Estabilidade: um unico socket persistente (sem TCP novo por poll),
//  servidor fixado no core 0, estado compartilhado protegido por spinlock,
//  aceita 1 WS ativo (fecha o anterior em reload), lru_purge nos sockets.
// ============================================================
#include "wifi_controle.h"
#include "config.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "esp_http_server.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

static const char* TAG = "WIFI_AP";

// Limite da area util da mesa (cm). Mesa 19 cm -> +-9.5; margem de 0.3.
#define SP_LIM  9.2f

// ---- Estado compartilhado (core 1 escreve, core 0 le) ------
// Spinlock cross-core: secoes criticas curtas, sem log/alocacao dentro.
static portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;
static float g_x = 0.0f, g_y = 0.0f, g_fps = 0.0f;
static bool  g_achou = false;
static float g_sp_x = SETPOINT_X_PADRAO, g_sp_y = SETPOINT_Y_PADRAO;

void wifi_controle_atualiza_posicao(float x, float y, bool achou, float fps) {
    portENTER_CRITICAL(&g_mux);
    g_x = x; g_y = y; g_achou = achou; g_fps = fps;
    portEXIT_CRITICAL(&g_mux);
}

void wifi_controle_le_setpoint(float* x, float* y) {
    portENTER_CRITICAL(&g_mux);
    *x = g_sp_x; *y = g_sp_y;
    portEXIT_CRITICAL(&g_mux);
}

void wifi_controle_setpoint(float x, float y) {
    if (x < -SP_LIM) x = -SP_LIM; else if (x > SP_LIM) x = SP_LIM;
    if (y < -SP_LIM) y = -SP_LIM; else if (y > SP_LIM) y = SP_LIM;
    portENTER_CRITICAL(&g_mux);
    g_sp_x = x; g_sp_y = y;
    portEXIT_CRITICAL(&g_mux);
}

// Monta o JSON de status num buffer; retorna o tamanho.
static int monta_status(char* buf, size_t n) {
    float x, y, fps, sx, sy; bool achou;
    portENTER_CRITICAL(&g_mux);
    x = g_x; y = g_y; fps = g_fps; achou = g_achou; sx = g_sp_x; sy = g_sp_y;
    portEXIT_CRITICAL(&g_mux);
    return snprintf(buf, n,
        "{\"x\":%.2f,\"y\":%.2f,\"a\":%d,\"f\":%.1f,\"sx\":%.2f,\"sy\":%.2f}",
        x, y, achou ? 1 : 0, fps, sx, sy);
}

// ---- Estado do servidor / WebSocket ------------------------
static httpd_handle_t g_server = NULL;
static int  g_ws_fd        = -1;   // fd do unico WS ativo (-1 = nenhum)
static int  g_sockets_ativos = 0;
static int  g_ultimo_erro  = 0;

// ---- Pagina HTML + Canvas ----------------------------------
// Mesa: 19x19 cm, coordenadas -9.5..+9.5 cm a partir do centro.
// Canvas: 280x280 px, margem 20 px -> area da mesa = 240x240 px.
static const char HTML[] =
    "<!DOCTYPE html><html><head>"
    "<meta charset=utf-8>"
    "<meta name=viewport content='width=device-width,initial-scale=1'>"
    "<title>Mesa PID</title>"
    "<style>"
    "body{margin:0;background:#111;display:flex;flex-direction:column;"
         "align-items:center;justify-content:center;min-height:100vh;"
         "font-family:monospace;color:#aaa;gap:14px}"
    "h2{font-size:1em;letter-spacing:3px;color:#555;margin:0}"
    "canvas{cursor:crosshair}"
    "#info{font-size:.85em;color:#555;min-height:1.2em}"
    "</style></head><body>"
    "<h2>MESA PID</h2>"
    "<canvas id=c width=280 height=280></canvas>"
    "<div id=info>conectando...</div>"
    "<script>"
    "var cv=document.getElementById('c'),"
        "ctx=cv.getContext('2d'),"
        "W=280,M=20,S=240,"       // canvas / margem / lado da mesa em px
        "MESA=19.0,sc=S/MESA,"    // px por cm
        "BR=2.0*sc,"              // raio da bola em px (2 cm = metade de 4 cm)
        "sp={x:0,y:0};"           // setpoint atual (para desenhar a mira)

    "function draw(x,y,a,fps){"
      "ctx.fillStyle='#111';ctx.fillRect(0,0,W,W);"
      "ctx.strokeStyle='#3a3a3a';ctx.lineWidth=2;"
      "ctx.strokeRect(M,M,S,S);"
      "ctx.strokeStyle='#1f1f1f';ctx.lineWidth=1;"
      "for(var g=-7.5;g<=7.5;g+=5){"
        "var px=W/2+g*sc,py=W/2+g*sc;"
        "ctx.beginPath();ctx.moveTo(px,M);ctx.lineTo(px,M+S);ctx.stroke();"
        "ctx.beginPath();ctx.moveTo(M,py);ctx.lineTo(M+S,py);ctx.stroke();"
      "}"
      "ctx.strokeStyle='#2e2e2e';ctx.lineWidth=1;"
      "ctx.beginPath();"
      "ctx.moveTo(W/2,M);ctx.lineTo(W/2,M+S);"
      "ctx.moveTo(M,W/2);ctx.lineTo(M+S,W/2);"
      "ctx.stroke();"
      "var spx=W/2+sp.x*sc,spy=W/2-sp.y*sc;"
      "ctx.strokeStyle='#fa0';ctx.lineWidth=1.5;"
      "ctx.beginPath();"
      "ctx.moveTo(spx-10,spy);ctx.lineTo(spx+10,spy);"
      "ctx.moveTo(spx,spy-10);ctx.lineTo(spx,spy+10);"
      "ctx.stroke();"
      "ctx.beginPath();ctx.arc(spx,spy,5,0,2*Math.PI);ctx.stroke();"
      "if(a){"
        "var cx=W/2+x*sc,cy=W/2-y*sc;"
        "cx=Math.max(M+BR,Math.min(M+S-BR,cx));"
        "cy=Math.max(M+BR,Math.min(M+S-BR,cy));"
        "ctx.beginPath();ctx.arc(cx,cy,BR,0,2*Math.PI);"
        "ctx.fillStyle='#f80';ctx.fill();"
        "ctx.strokeStyle='#fa0';ctx.lineWidth=1.5;ctx.stroke();"
      "}"
      "document.getElementById('info').textContent="
        "a?('x: '+x.toFixed(1)+'  y: '+y.toFixed(1)+'  alvo: ('+sp.x.toFixed(1)+','+sp.y.toFixed(1)+')')"
          ":'sem bola';"
    "}"

    "draw(0,0,false,0);"

    // ---- WebSocket: 1 conexao persistente, tick a cada 50ms ----
    // Robusto a socket meio-aberto: watchdog forca reconexao se parar de
    // chegar dado, e reconecta quando o app volta ao foco (tela liga).
    "var ws,tickTimer,wdTimer,backoff=300,lastRx=0;"
    "function limpa(){"
      "clearInterval(tickTimer);tickTimer=null;"
      "clearInterval(wdTimer);wdTimer=null;"
    "}"
    "function connect(){"
      "limpa();"
      "try{ws=new WebSocket('ws://'+location.host+'/ws');}catch(_){"
        "setTimeout(connect,backoff);return;}"
      "ws.onopen=function(){"
        "backoff=300;lastRx=Date.now();"
        "document.getElementById('info').textContent='conectado';"
        "tickTimer=setInterval(function(){"
          "if(ws&&ws.readyState===1)ws.send('{\"cmd\":\"tick\"}');"
        "},50);"           // 20 Hz
        "wdTimer=setInterval(function(){"   // sem resposta 1.5s -> derruba
          "if(Date.now()-lastRx>1500){try{ws.close();}catch(_){}}"
        "},500);"
      "};"
      "ws.onmessage=function(e){"
        "lastRx=Date.now();"
        "try{var d=JSON.parse(e.data);}catch(_){return;}"
        "if(d.sx!==undefined)sp={x:d.sx,y:d.sy};"
        "if(d.x!==undefined)draw(d.x,d.y,d.a===1,d.f);"
      "};"
      "ws.onclose=function(){"
        "limpa();"
        "document.getElementById('info').textContent='reconectando...';"
        "setTimeout(connect,backoff);"
        "backoff=Math.min(backoff*2,1500);"   // 300 -> 600 -> 1200 -> 1500
      "};"
      "ws.onerror=function(){try{ws.close();}catch(_){}};"
    "}"
    "connect();"
    // app voltou ao foco e o socket nao esta vivo -> reconecta ja
    "document.addEventListener('visibilitychange',function(){"
      "if(!document.hidden&&(!ws||ws.readyState>1))connect();"
    "});"

    // click na mesa -> novo setpoint pelo mesmo socket
    "cv.addEventListener('click',function(e){"
      "var r=cv.getBoundingClientRect(),"
          "px=(e.clientX-r.left)*W/r.width,"
          "py=(e.clientY-r.top)*W/r.height;"
      "var x=(px-W/2)/sc,y=-(py-W/2)/sc;"
      "var lim=MESA/2-0.3;"
      "x=Math.max(-lim,Math.min(lim,x));"
      "y=Math.max(-lim,Math.min(lim,y));"
      "sp={x:x,y:y};"
      "if(ws&&ws.readyState===1)"
        "ws.send('{\"cmd\":\"sp\",\"x\":'+x.toFixed(2)+',\"y\":'+y.toFixed(2)+'}');"
    "});"
    "</script></body></html>";

// ---- Handler GET / -----------------------------------------
static esp_err_t handler_root(httpd_req_t* req) {
    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");   // nunca servir HTML velho
    return httpd_resp_send(req, HTML, strlen(HTML));
}

// ---- Handler GET /favicon.ico (evita 404 + socket a toa) ---
static esp_err_t handler_favicon(httpd_req_t* req) {
    httpd_resp_set_status(req, "204 No Content");
    return httpd_resp_send(req, NULL, 0);
}

// ---- Handler GET /health (diagnostico) ---------------------
static esp_err_t handler_health(httpd_req_t* req) {
    char buf[192];
    int len = snprintf(buf, sizeof(buf),
        "{\"uptime_s\":%lld,\"heap\":%u,\"heap_min\":%u,"
        "\"sockets\":%d,\"ws\":%d,\"last_err\":%d}",
        (long long)(esp_timer_get_time() / 1000000),
        (unsigned)esp_get_free_heap_size(),
        (unsigned)esp_get_minimum_free_heap_size(),
        g_sockets_ativos, g_ws_fd >= 0 ? 1 : 0, g_ultimo_erro);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, buf, len);
}

// Responde o status atual pelo WebSocket aberto.
static esp_err_t ws_envia_status(httpd_req_t* req) {
    char buf[96];
    int len = monta_status(buf, sizeof(buf));
    httpd_ws_frame_t out = {};
    out.type    = HTTPD_WS_TYPE_TEXT;
    out.payload = (uint8_t*)buf;
    out.len     = len;
    return httpd_ws_send_frame(req, &out);
}

// ---- Handler GET /ws (WebSocket) ---------------------------
static esp_err_t handler_ws(httpd_req_t* req) {
    if (req->method == HTTP_GET) {
        // Handshake concluido. Aceita 1 WS ativo: derruba o anterior.
        int fd = httpd_req_to_sockfd(req);
        if (g_ws_fd >= 0 && g_ws_fd != fd) {
            ESP_LOGI(TAG, "novo WS (fd=%d), fechando anterior (fd=%d)", fd, g_ws_fd);
            httpd_sess_trigger_close(g_server, g_ws_fd);
        }
        g_ws_fd = fd;
        ESP_LOGI(TAG, "WS conectado (fd=%d)", fd);
        return ESP_OK;
    }

    // Le o tamanho do frame (payload=NULL, len=0).
    httpd_ws_frame_t frame = {};
    frame.type = HTTPD_WS_TYPE_TEXT;
    esp_err_t ret = httpd_ws_recv_frame(req, &frame, 0);
    if (ret != ESP_OK) { g_ultimo_erro = ret; return ret; }

    if (frame.type == HTTPD_WS_TYPE_CLOSE) {
        ESP_LOGI(TAG, "WS close (fd=%d)", httpd_req_to_sockfd(req));
        return ESP_OK;
    }
    if (frame.type != HTTPD_WS_TYPE_TEXT || frame.len == 0) return ESP_OK;
    if (frame.len > 127) return ESP_OK;   // mensagens nossas sao curtas

    uint8_t payload[128];
    frame.payload = payload;
    ret = httpd_ws_recv_frame(req, &frame, sizeof(payload) - 1);
    if (ret != ESP_OK) { g_ultimo_erro = ret; return ret; }
    payload[frame.len] = 0;

    // {"cmd":"sp","x":..,"y":..}  -> atualiza setpoint
    if (strstr((char*)payload, "\"sp\"")) {
        char* px = strstr((char*)payload, "\"x\":");
        char* py = strstr((char*)payload, "\"y\":");
        if (px && py) {
            float x = strtof(px + 4, NULL);
            float y = strtof(py + 4, NULL);
            wifi_controle_setpoint(x, y);   // ja faz clamp
        }
    }
    // tick e sp respondem o mesmo snapshot de status.
    return ws_envia_status(req);
}

// Chamado quando uma sessao (qualquer socket) fecha.
static void on_sess_close(httpd_handle_t hd, int sockfd) {
    if (sockfd == g_ws_fd) {
        g_ws_fd = -1;
        ESP_LOGI(TAG, "WS desconectado (fd=%d)", sockfd);
    }
    if (g_sockets_ativos > 0) g_sockets_ativos--;
    close(sockfd);   // close_fn customizado precisa fechar o socket
}

// Chamado quando uma nova sessao abre.
static esp_err_t on_sess_open(httpd_handle_t hd, int sockfd) {
    g_sockets_ativos++;
    return ESP_OK;
}

// ---- Eventos do AP -----------------------------------------
static void wifi_evt(void* arg, esp_event_base_t base, int32_t id, void* data) {
    if (id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t* e = (wifi_event_ap_staconnected_t*)data;
        ESP_LOGI(TAG, "celular conectou (aid=%d)", e->aid);
    } else if (id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t* e = (wifi_event_ap_stadisconnected_t*)data;
        ESP_LOGI(TAG, "celular saiu (aid=%d)", e->aid);
    }
}

// ---- Ponto de entrada --------------------------------------
void wifi_controle_inicia() {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t wcfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wcfg));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                               wifi_evt, NULL));

    wifi_config_t ap = {};
    strncpy((char*)ap.ap.ssid,  WIFI_AP_SSID, sizeof(ap.ap.ssid) - 1);
    ap.ap.ssid_len       = (uint8_t)strlen(WIFI_AP_SSID);
    ap.ap.channel        = 6;
    ap.ap.authmode       = WIFI_AUTH_OPEN;
    ap.ap.max_connection = 2;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap));
    ESP_ERROR_CHECK(esp_wifi_start());

    // SoftAP responsivo: desliga o modem-sleep. Com power-save (padrao) o AP
    // bufferiza pacotes e o timing fica erratico -> lag, sockets meio-abertos
    // e o celular largando o AP sozinho. WIFI_PS_NONE estabiliza a conexao.
    esp_wifi_set_ps(WIFI_PS_NONE);

    httpd_config_t hcfg   = HTTPD_DEFAULT_CONFIG();
    hcfg.core_id          = 0;                       // junto do WiFi, longe da visao
    hcfg.task_priority    = tskIDLE_PRIORITY + 8;
    hcfg.stack_size       = 8192;
    hcfg.max_open_sockets = 7;                       // -3 reservados = 4 p/ cliente
                                                     // (pagina + WS + favicon + folga)
    hcfg.lru_purge_enable = true;                    // recicla socket mais antigo
    hcfg.recv_wait_timeout = 5;                      // s; tick a 50ms nunca estoura
    hcfg.send_wait_timeout = 5;
    hcfg.open_fn          = on_sess_open;
    hcfg.close_fn         = on_sess_close;

    ESP_ERROR_CHECK(httpd_start(&g_server, &hcfg));

    httpd_uri_t u_root = {"/",            HTTP_GET, handler_root,    NULL};
    httpd_uri_t u_fav  = {"/favicon.ico", HTTP_GET, handler_favicon, NULL};
    httpd_uri_t u_hlt  = {"/health",      HTTP_GET, handler_health,  NULL};
    httpd_uri_t u_ws   = {"/ws",          HTTP_GET, handler_ws,      NULL};
    u_ws.is_websocket  = true;
    httpd_register_uri_handler(g_server, &u_root);
    httpd_register_uri_handler(g_server, &u_fav);
    httpd_register_uri_handler(g_server, &u_hlt);
    httpd_register_uri_handler(g_server, &u_ws);

    ESP_LOGI(TAG, "AP '%s' ativo. Abra: http://192.168.4.1", WIFI_AP_SSID);
}
