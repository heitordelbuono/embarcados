// ============================================================
//  debug_visao.cpp - render de debug da visao
//  Desenho de overlays no frame RGB565 + envio do frame anotado pela
//  serial em PPM/base64 (lido por tools/recebe_debug_visao.py).
// ============================================================
#include "debug_visao.h"
#include "config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

// ---------- desenho (sempre RGB565 big-endian, igual ao frame buffer) ------
void desenha_put_px(uint8_t* buf, int x, int y, uint16_t cor) {
    if (x < 0 || x >= CAM_LARGURA || y < 0 || y >= CAM_ALTURA) return;
    int idx = y * CAM_LARGURA + x;
    buf[2 * idx] = (uint8_t)(cor >> 8);
    buf[2 * idx + 1] = (uint8_t)(cor & 0xFF);
}
static void linha_h(uint8_t* buf, int x0, int x1, int y, uint16_t cor) {
    if (x0 > x1) { int t = x0; x0 = x1; x1 = t; }
    for (int x = x0; x <= x1; x++) desenha_put_px(buf, x, y, cor);
}
static void linha_v(uint8_t* buf, int x, int y0, int y1, uint16_t cor) {
    if (y0 > y1) { int t = y0; y0 = y1; y1 = t; }
    for (int y = y0; y <= y1; y++) desenha_put_px(buf, x, y, cor);
}
static void linha(uint8_t* buf, int x0, int y0, int x1, int y1, uint16_t cor) {
    int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;
    for (;;) {
        desenha_put_px(buf, x0, y0, cor);
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}
void desenha_poligono(uint8_t* buf, const PontoF q[4], uint16_t cor) {
    for (int i = 0; i < 4; i++) {
        int j = (i + 1) & 3;
        linha(buf, (int)lroundf(q[i].x), (int)lroundf(q[i].y),
              (int)lroundf(q[j].x), (int)lroundf(q[j].y), cor);
    }
}
void desenha_retangulo(uint8_t* buf, int x0, int y0, int x1, int y1, uint16_t cor) {
    linha_h(buf, x0, x1, y0, cor); linha_h(buf, x0, x1, y1, cor);
    linha_v(buf, x0, y0, y1, cor); linha_v(buf, x1, y0, y1, cor);
}
void desenha_cruz(uint8_t* buf, int x, int y, uint16_t cor) {
    linha_h(buf, x - 4, x + 4, y, cor); linha_v(buf, x, y - 4, y + 4, cor);
}

void overlay_sobel(uint8_t* buf, const camera_fb_t* fb) {
    uint8_t linha_ant[CAM_LARGURA];
    uint8_t linha_at[CAM_LARGURA];

    for (int x = 0; x < CAM_LARGURA; x++) linha_ant[x] = pixel_luma(fb, x);

    for (int y = 1; y < CAM_ALTURA - 1; y++) {
        for (int x = 0; x < CAM_LARGURA; x++) linha_at[x] = pixel_luma(fb, y * CAM_LARGURA + x);

        for (int x = 1; x < CAM_LARGURA - 1; x++) {
            int i = y * CAM_LARGURA + x;
            int luma_tl = linha_ant[x - 1], luma_tc = linha_ant[x], luma_tr = linha_ant[x + 1];
            int luma_l = linha_at[x - 1], luma_r = linha_at[x + 1];
            int luma_bl = pixel_luma(fb, i + CAM_LARGURA - 1);
            int luma_bc = pixel_luma(fb, i + CAM_LARGURA);
            int luma_br = pixel_luma(fb, i + CAM_LARGURA + 1);

            int gx = -luma_tl + luma_tr - 2 * luma_l + 2 * luma_r - luma_bl + luma_br;
            int gy = -luma_tl - 2 * luma_tc - luma_tr + luma_bl + 2 * luma_bc + luma_br;

            if (abs(gx) + abs(gy) >= BORDA_GRAD_MIN * 8) desenha_put_px(buf, x, y, 0xFFFF);
        }
        memcpy(linha_ant, linha_at, CAM_LARGURA);
    }
}

// ---------- base64 (frame de debug pela serial) ----------
static void b64_triplet(uint8_t a, uint8_t b, uint8_t c, int n) {
    static const char* T = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    char o[5];
    uint32_t v = ((uint32_t)a << 16) | ((uint32_t)b << 8) | c;
    o[0] = T[(v >> 18) & 63]; o[1] = T[(v >> 12) & 63];
    o[2] = (n >= 2) ? T[(v >> 6) & 63] : '='; o[3] = (n >= 3) ? T[v & 63] : '='; o[4] = 0;
    printf("%s", o);
}
class Base64Stream {
public:
    void put(uint8_t byte) {
        trio[n++] = byte;
        if (n == 3) {
            b64_triplet(trio[0], trio[1], trio[2], 3);
            col += 4;
            n = 0;
            if (col >= 76) {
                printf("\n");
                col = 0;
                linhas++;
                if (linhas % 20 == 0) {
                    vTaskDelay(pdMS_TO_TICKS(10)); // cede a CPU para a IDLE resetar o watchdog
                }
            }
        }
    }
    void finish() {
        if (n == 1) b64_triplet(trio[0], 0, 0, 1);
        else if (n == 2) b64_triplet(trio[0], trio[1], 0, 2);
        if (n || col) printf("\n");
        n = 0; col = 0; linhas = 0;
        vTaskDelay(pdMS_TO_TICKS(5));
    }
private:
    uint8_t trio[3] = {}; int n = 0; int col = 0; int linhas = 0;
};

void envia_ppm_base64(const camera_fb_t* fb, const VisaoDebugInfo& d) {
    printf("\n===DEBUG_VISAO_META achou=%d cx=%.2f cy=%.2f mesa_cm=%.2f,%.2f filt_cm=%.2f,%.2f vel=%.2f,%.2f area=%d blobs=%d runs=%d overflow=%d fundo=%d bbox=%d,%d,%d,%d roi_bbox=%d,%d,%d,%d media_y=%d dt_us=%lld===\n",
           d.achou ? 1 : 0, d.cx_f, d.cy_f, d.mesa_x_cm, d.mesa_y_cm, d.filt_x_cm, d.filt_y_cm,
           d.vel_x_cm, d.vel_y_cm, d.area, d.blobs, d.runs, d.overflow ? 1 : 0, d.usou_fundo ? 1 : 0,
           d.x0, d.y0, d.x1, d.y1, d.roi_x0, d.roi_y0, d.roi_x1, d.roi_y1,
           d.media_y, (long long)d.dt_us);
    printf("===DEBUG_VISAO_BASE64_INICIO===\n");
    Base64Stream b64;
    char header[40];
    int hl = snprintf(header, sizeof(header), "P6\n%d %d\n255\n", (int)fb->width, (int)fb->height);
    for (int i = 0; i < hl; i++) b64.put((uint8_t)header[i]);
    int pixels = fb->width * fb->height;
    for (int i = 0; i < pixels; i++) {
        int r, g, b; rgb565_split(rgb565_at(fb->buf, i), r, g, b);
        b64.put((uint8_t)r); b64.put((uint8_t)g); b64.put((uint8_t)b);
    }
    b64.finish();
    printf("===DEBUG_VISAO_BASE64_FIM===\n");
}
