// ============================================================
//  wifi_controle.cpp
//  AP "MesaPID" + HTTP server.
//    GET /    -> pagina HTML com canvas animado
//    GET /s   -> JSON com posicao (o browser consulta a cada 100ms)
//    GET /sp?x=1.2&y=-0.5 -> atualiza setpoint do PID
// ============================================================
#include "wifi_controle.h"
#include "config.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_http_server.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

static const char* TAG = "WIFI_AP";

volatile PosicaoWeb g_pos_web    = {0.0f, 0.0f, 0.0f, false};
volatile float      g_setpoint_x = SETPOINT_X_PADRAO;
volatile float      g_setpoint_y = SETPOINT_Y_PADRAO;

// ---- Pagina HTML + Canvas ----------------------------------
// Mesa: 19x19 cm, coordenadas -9.5..+9.5 cm a partir do centro.
// Canvas: 280x280 px, margem 20 px -> area da mesa = 240x240 px.
// Escala: 240/19 = ~12.6 px/cm. Bola 4 cm diametro -> raio ~25 px.
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
    "<div id=info>aguardando...</div>"
    "<script>"
    "var cv=document.getElementById('c'),"
        "ctx=cv.getContext('2d'),"
        "W=280,M=20,S=240,"       // canvas / margem / lado da mesa em px
        "MESA=19.0,sc=S/MESA,"    // px por cm
        "BR=2.0*sc,"              // raio da bola em px (2 cm = metade de 4 cm)
        "sp={x:0,y:0};"           // setpoint atual (para desenhar a mira)

    "function draw(x,y,a,fps){"
      // fundo
      "ctx.fillStyle='#111';ctx.fillRect(0,0,W,W);"
      // mesa
      "ctx.strokeStyle='#3a3a3a';ctx.lineWidth=2;"
      "ctx.strokeRect(M,M,S,S);"
      // grade leve (cada 5 cm)
      "ctx.strokeStyle='#1f1f1f';ctx.lineWidth=1;"
      "for(var g=-7.5;g<=7.5;g+=5){"
        "var px=W/2+g*sc,py=W/2+g*sc;"
        "ctx.beginPath();ctx.moveTo(px,M);ctx.lineTo(px,M+S);ctx.stroke();"
        "ctx.beginPath();ctx.moveTo(M,py);ctx.lineTo(M+S,py);ctx.stroke();"
      "}"
      // eixos centrais
      "ctx.strokeStyle='#2e2e2e';ctx.lineWidth=1;"
      "ctx.beginPath();"
      "ctx.moveTo(W/2,M);ctx.lineTo(W/2,M+S);"
      "ctx.moveTo(M,W/2);ctx.lineTo(M+S,W/2);"
      "ctx.stroke();"
      // mira do setpoint (amarela)
      "var spx=W/2+sp.x*sc,spy=W/2-sp.y*sc;"
      "ctx.strokeStyle='#fa0';ctx.lineWidth=1.5;"
      "ctx.beginPath();"
      "ctx.moveTo(spx-10,spy);ctx.lineTo(spx+10,spy);"
      "ctx.moveTo(spx,spy-10);ctx.lineTo(spx,spy+10);"
      "ctx.stroke();"
      "ctx.beginPath();ctx.arc(spx,spy,5,0,2*Math.PI);ctx.stroke();"
      // bola
      "if(a){"
        "var cx=W/2+x*sc,cy=W/2-y*sc;"
        "cx=Math.max(M+BR,Math.min(M+S-BR,cx));"
        "cy=Math.max(M+BR,Math.min(M+S-BR,cy));"
        "ctx.beginPath();ctx.arc(cx,cy,BR,0,2*Math.PI);"
        "ctx.fillStyle='#f80';ctx.fill();"
        "ctx.strokeStyle='#fa0';ctx.lineWidth=1.5;ctx.stroke();"
      "}"
      // info
      "document.getElementById('info').textContent="
        "a?('x: '+x.toFixed(1)+'  y: '+y.toFixed(1)+'  alvo: ('+sp.x.toFixed(1)+','+sp.y.toFixed(1)+')')"
          ":'sem bola';"
    "}"

    "draw(0,0,false,0);"

    // click na mesa -> novo setpoint
    "cv.addEventListener('click',function(e){"
      "var r=cv.getBoundingClientRect(),"
          "px=(e.clientX-r.left)*W/r.width,"
          "py=(e.clientY-r.top)*W/r.height;"
      // converte px -> cm (Y invertido)
      "var x=(px-W/2)/sc,y=-(py-W/2)/sc;"
      // limita a area da mesa
      "var lim=MESA/2-0.3;"
      "x=Math.max(-lim,Math.min(lim,x));"
      "y=Math.max(-lim,Math.min(lim,y));"
      "sp={x:x,y:y};"
      "fetch('/sp?x='+x.toFixed(2)+'&y='+y.toFixed(2));"
    "});"

    // polling: pega a posicao a cada 100 ms (nao segura o servidor como SSE)
    "setInterval(function(){"
      "fetch('/s').then(function(r){return r.json()}).then(function(d){"
        "if(d.sx!==undefined)sp={x:d.sx,y:d.sy};"
        "draw(d.x,d.y,d.a===1,d.f);"
      "}).catch(function(){"
        "document.getElementById('info').textContent='reconectando...';"
      "});"
    "},100);"
    "</script></body></html>";

// ---- Handler GET / -----------------------------------------
static esp_err_t handler_root(httpd_req_t* req) {
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, HTML, strlen(HTML));
}

// ---- Handler GET /s  (status JSON) -------------------------
static esp_err_t handler_status(httpd_req_t* req) {
    char buf[96];
    int len = snprintf(buf, sizeof(buf),
        "{\"x\":%.2f,\"y\":%.2f,\"a\":%d,\"f\":%.1f,\"sx\":%.2f,\"sy\":%.2f}",
        (float)g_pos_web.x, (float)g_pos_web.y, g_pos_web.achou ? 1 : 0,
        (float)g_pos_web.fps, (float)g_setpoint_x, (float)g_setpoint_y);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, buf, len);
}

// ---- Handler GET /sp?x=1.2&y=-0.5 -------------------------
static esp_err_t handler_setpoint(httpd_req_t* req) {
    char query[64];
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
        char val[16];
        if (httpd_query_key_value(query, "x", val, sizeof(val)) == ESP_OK)
            g_setpoint_x = strtof(val, NULL);
        if (httpd_query_key_value(query, "y", val, sizeof(val)) == ESP_OK)
            g_setpoint_y = strtof(val, NULL);
        ESP_LOGI(TAG, "setpoint -> (%.2f, %.2f)", (float)g_setpoint_x, (float)g_setpoint_y);
    }
    return httpd_resp_send(req, "ok", 2);
}

// ---- Ponto de entrada --------------------------------------
void wifi_controle_inicia() {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t wcfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wcfg));

    wifi_config_t ap = {};
    strncpy((char*)ap.ap.ssid,  WIFI_AP_SSID, sizeof(ap.ap.ssid) - 1);
    ap.ap.ssid_len       = (uint8_t)strlen(WIFI_AP_SSID);
    ap.ap.channel        = 6;
    ap.ap.authmode       = WIFI_AUTH_OPEN;
    ap.ap.max_connection = 2;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap));
    ESP_ERROR_CHECK(esp_wifi_start());

    httpd_config_t hcfg   = HTTPD_DEFAULT_CONFIG();
    hcfg.stack_size       = 8192;
    hcfg.max_open_sockets = 4;
    httpd_handle_t server = NULL;
    ESP_ERROR_CHECK(httpd_start(&server, &hcfg));

    httpd_uri_t u_root = {"/",  HTTP_GET, handler_root,     NULL};
    httpd_uri_t u_st   = {"/s", HTTP_GET, handler_status,   NULL};
    httpd_uri_t u_sp   = {"/sp", HTTP_GET, handler_setpoint, NULL};
    httpd_register_uri_handler(server, &u_root);
    httpd_register_uri_handler(server, &u_st);
    httpd_register_uri_handler(server, &u_sp);

    ESP_LOGI(TAG, "AP '%s' ativo. Abra: http://192.168.4.1", WIFI_AP_SSID);
}
