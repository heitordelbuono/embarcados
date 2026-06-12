// ============================================================
//  visao.cpp - Deteccao da bola (pipeline otimizado)
//
//  Etapas do processamento (ver docs/VISAO.md):
//   1. Captura grayscale QQVGA (sensor com exposicao/ganho/AWB FIXOS).
//      Opcional: captura em JPEG e DECODIFICA on-chip p/ RGB565 (CAM_CAPTURA_JPEG).
//   2. Referencia local: grade de iluminacao (4x4) OU fundo da mesa vazia.
//   3. Candidatos: luma - referencia >= delta  (e acima do piso absoluto).
//   4. Componentes conexos: run-length + union-find numa unica passada.
//   5. Escolhe o melhor blob por SCORE (forma + proximidade da ultima pos).
//   6. Centroide ponderado pela intensidade (sub-pixel) + refino opcional.
//   7. Homografia 3x3 (pre-computada) projeta pixel -> cm no plano da mesa.
//   8. Filtro alfa-beta: suaviza a posicao e estima a velocidade (cm/s).
// ============================================================
#include "visao.h"
#include "config.h"
#include "esp_camera.h"
#include "img_converters.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const char* TAG = "VISAO";

static const int MAX_PIXELS = CAM_LARGURA * CAM_ALTURA;
static const int SCAN_STEP = VISAO_SCAN_STEP < 1 ? 1 : VISAO_SCAN_STEP;
static const int MEDIA_INTERVALO = VISAO_MEDIA_INTERVALO < 1 ? 1 : VISAO_MEDIA_INTERVALO;

// ---------- buffers (PSRAM) ----------
static uint8_t* s_bg = NULL;          // fundo da mesa vazia (referencia opcional)
static bool     s_bg_valido = false;
static bool     s_frame_grayscale = false;
static uint8_t* s_rgb565 = NULL;      // matriz RGB565 decodificada do JPEG
static bool     s_modo_jpeg = (CAM_CAPTURA_JPEG != 0);  // modo de captura (runtime)

struct PontoF { float x; float y; };

static PontoF s_mesa_ext[4];          // cantos da mesa (px, resolucao de deteccao)
static PontoF s_mesa_roi[4];          // ROI util dentro da mesa (px)

// homografia px->cm (8 coef; h22 = 1)
static float s_H[8];
static bool  s_H_ok = false;
static float s_centro_dx = 0.0f;       // offset manual da origem em pixels
static float s_centro_dy = 0.0f;

// LUT de varredura: faixa [x0,x1] da ROI por linha (pre-computada)
static int16_t s_roi_x0[CAM_ALTURA];
static int16_t s_roi_x1[CAM_ALTURA];

// bbox da ROI (pre-computado)
static int s_mb_x0, s_mb_y0, s_mb_x1, s_mb_y1;

// grade de iluminacao local (referencia adaptativa)
static int s_grid[GRADE_NY][GRADE_NX];
static bool s_grid_ok = false;
static int s_media_mesa = 0;          // brilho medio global da ROI (informativo)

// LUT de celula por coluna/linha -> elimina divisao por pixel na referencia
static uint8_t s_cellx[CAM_LARGURA];
static uint8_t s_celly[CAM_ALTURA];

// componentes conexos
struct RunAcc {
    int16_t x0, x1;             // extensao horizontal (px) nesta linha
    int16_t bx0, bx1, by0, by1; // bbox acumulada
    int32_t area;              // contagem de pixels (na grade de varredura)
    int64_t sx, sy;            // somas ponderadas (x*peso, y*peso)
    int64_t wsum;              // soma dos pesos
    int32_t parent;            // union-find
};
static RunAcc* s_runs = NULL;

// ============================================================
//  Calibracao geometrica + homografia
// ============================================================
static PontoF escala_calib(float x, float y) {
    return { x * ((float)CAM_LARGURA / MESA_CALIB_LARGURA),
             y * ((float)CAM_ALTURA  / MESA_CALIB_ALTURA) };
}

static void prepara_calibracao_mesa() {
    s_mesa_ext[0] = escala_calib((float)MESA_EXT_TL_X, (float)MESA_EXT_TL_Y);
    s_mesa_ext[1] = escala_calib((float)MESA_EXT_TR_X, (float)MESA_EXT_TR_Y);
    s_mesa_ext[2] = escala_calib((float)MESA_EXT_BR_X, (float)MESA_EXT_BR_Y);
    s_mesa_ext[3] = escala_calib((float)MESA_EXT_BL_X, (float)MESA_EXT_BL_Y);

    s_mesa_roi[0] = escala_calib((float)MESA_ROI_TL_X, (float)MESA_ROI_TL_Y);
    s_mesa_roi[1] = escala_calib((float)MESA_ROI_TR_X, (float)MESA_ROI_TR_Y);
    s_mesa_roi[2] = escala_calib((float)MESA_ROI_BR_X, (float)MESA_ROI_BR_Y);
    s_mesa_roi[3] = escala_calib((float)MESA_ROI_BL_X, (float)MESA_ROI_BL_Y);
}

// Resolve A x = b para N=8 por eliminacao de Gauss com pivoteamento parcial.
static bool resolve8(float A[8][8], float b[8], float x[8]) {
    const int N = 8;
    for (int col = 0; col < N; col++) {
        int piv = col;
        float melhor = fabsf(A[col][col]);
        for (int r = col + 1; r < N; r++) {
            if (fabsf(A[r][col]) > melhor) { melhor = fabsf(A[r][col]); piv = r; }
        }
        if (melhor < 1e-9f) return false;
        if (piv != col) {
            for (int c = 0; c < N; c++) { float t = A[col][c]; A[col][c] = A[piv][c]; A[piv][c] = t; }
            float t = b[col]; b[col] = b[piv]; b[piv] = t;
        }
        for (int r = 0; r < N; r++) {
            if (r == col) continue;
            float f = A[r][col] / A[col][col];
            if (f == 0.0f) continue;
            for (int c = col; c < N; c++) A[r][c] -= f * A[col][c];
            b[r] -= f * b[col];
        }
    }
    for (int i = 0; i < N; i++) x[i] = b[i] / A[i][i];
    return true;
}

// Homografia que mapeia (u,v) px -> (X,Y) cm, origem no centro,
// +X para a esquerda, +Y para cima (igual ao diagrama do projeto).
static void prepara_homografia() {
    const float h = MESA_LADO_CM * 0.5f;
    const float wX[4] = { +h, -h, -h, +h };   // TL TR BR BL
    const float wY[4] = { +h, +h, -h, -h };

    float A[8][8] = {};
    float b[8] = {};
    for (int i = 0; i < 4; i++) {
        float u = s_mesa_ext[i].x, v = s_mesa_ext[i].y;
        float X = wX[i], Y = wY[i];
        int r = 2 * i;
        A[r][0] = u; A[r][1] = v; A[r][2] = 1; A[r][6] = -u * X; A[r][7] = -v * X; b[r] = X;
        r++;
        A[r][3] = u; A[r][4] = v; A[r][5] = 1; A[r][6] = -u * Y; A[r][7] = -v * Y; b[r] = Y;
    }
    s_H_ok = resolve8(A, b, s_H);
    if (!s_H_ok) ESP_LOGE(TAG, "homografia degenerada — confira os cantos MESA_EXT_*");
}

static void homografia_px_para_cm(float px, float py, float& x_cm, float& y_cm) {
    if (!s_H_ok) { x_cm = 0; y_cm = 0; return; }
    float w = s_H[6] * px + s_H[7] * py + 1.0f;
    if (fabsf(w) < 1e-6f) w = 1e-6f;
    x_cm = (s_H[0] * px + s_H[1] * py + s_H[2]) / w;
    y_cm = (s_H[3] * px + s_H[4] * py + s_H[5]) / w;
}

static void pixel_para_cm(float px, float py, float& x_cm, float& y_cm) {
    homografia_px_para_cm(px - s_centro_dx, py - s_centro_dy, x_cm, y_cm);
}

static PontoF centro_mesa_px() {
    return { (s_mesa_ext[0].x + s_mesa_ext[1].x + s_mesa_ext[2].x + s_mesa_ext[3].x) * 0.25f,
             (s_mesa_ext[0].y + s_mesa_ext[1].y + s_mesa_ext[2].y + s_mesa_ext[3].y) * 0.25f };
}

static PontoF centro_origem_px() {
    PontoF c = centro_mesa_px();
    c.x += s_centro_dx;
    c.y += s_centro_dy;
    return c;
}

// ============================================================
//  Utilidades
// ============================================================
static int clamp_int(int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); }
#if FILTRO_GATING_ATIVO
static float distf(float ax, float ay, float bx, float by) {
    float dx = ax - bx, dy = ay - by; return sqrtf(dx * dx + dy * dy);
}
#endif

static void bbox_quad(const PontoF q[4], int& x0, int& y0, int& x1, int& y1) {
    float minx = q[0].x, maxx = q[0].x, miny = q[0].y, maxy = q[0].y;
    for (int i = 1; i < 4; i++) {
        if (q[i].x < minx) minx = q[i].x;
        if (q[i].x > maxx) maxx = q[i].x;
        if (q[i].y < miny) miny = q[i].y;
        if (q[i].y > maxy) maxy = q[i].y;
    }
    x0 = clamp_int((int)floorf(minx), 0, CAM_LARGURA - 1);
    y0 = clamp_int((int)floorf(miny), 0, CAM_ALTURA - 1);
    x1 = clamp_int((int)ceilf(maxx),  0, CAM_LARGURA - 1);
    y1 = clamp_int((int)ceilf(maxy),  0, CAM_ALTURA - 1);
}

static bool faixa_linha_quad(const PontoF q[4], int y, int& x0, int& x1) {
    float sy = (float)y + 0.5f;
    float xs[4];
    int n = 0;
    for (int i = 0; i < 4; i++) {
        PontoF a = q[i], b = q[(i + 1) & 3];
        bool cruza = (a.y <= sy && b.y > sy) || (b.y <= sy && a.y > sy);
        if (!cruza) continue;
        float t = (sy - a.y) / (b.y - a.y);
        xs[n++] = a.x + t * (b.x - a.x);
        if (n >= 4) break;
    }
    if (n < 2) return false;
    if (xs[0] > xs[1]) { float t = xs[0]; xs[0] = xs[1]; xs[1] = t; }
    x0 = clamp_int((int)ceilf(xs[0]),  0, CAM_LARGURA - 1);
    x1 = clamp_int((int)floorf(xs[1]), 0, CAM_LARGURA - 1);
    return x0 <= x1;
}

// Pre-computa LUTs uma vez: faixa da ROI por linha + indice de celula por x/y.
static void prepara_luts() {
#if VISAO_ROI_LIVRE
    // sem ROI: cada linha cobre a largura inteira; bbox = frame inteiro
    for (int y = 0; y < CAM_ALTURA; y++) { s_roi_x0[y] = 0; s_roi_x1[y] = CAM_LARGURA - 1; }
    s_mb_x0 = 0; s_mb_y0 = 0; s_mb_x1 = CAM_LARGURA - 1; s_mb_y1 = CAM_ALTURA - 1;
#else
    // poligono de busca: externo (mesa toda) ou interno (inset), conforme config
  #if VISAO_ROI_EXTERNO
    const PontoF* roiPoly = s_mesa_ext;
  #else
    const PontoF* roiPoly = s_mesa_roi;
  #endif
    for (int y = 0; y < CAM_ALTURA; y++) {
        int a, b;
        if (faixa_linha_quad(roiPoly, y, a, b)) { s_roi_x0[y] = a; s_roi_x1[y] = b; }
        else { s_roi_x0[y] = 1; s_roi_x1[y] = 0; } // vazio
    }
    bbox_quad(roiPoly, s_mb_x0, s_mb_y0, s_mb_x1, s_mb_y1);
#endif

    int mbw = s_mb_x1 - s_mb_x0 + 1;
    int mbh = s_mb_y1 - s_mb_y0 + 1;
    for (int x = 0; x < CAM_LARGURA; x++) {
        int c = (mbw > 0) ? (x - s_mb_x0) * GRADE_NX / mbw : 0;
        s_cellx[x] = (uint8_t)clamp_int(c, 0, GRADE_NX - 1);
    }
    for (int y = 0; y < CAM_ALTURA; y++) {
        int c = (mbh > 0) ? (y - s_mb_y0) * GRADE_NY / mbh : 0;
        s_celly[y] = (uint8_t)clamp_int(c, 0, GRADE_NY - 1);
    }
}

// ---------- leitura de pixel ----------
static uint16_t rgb565_at(const uint8_t* buf, int idx) {
    return ((uint16_t)buf[2 * idx] << 8) | buf[2 * idx + 1];
}
static void rgb565_split(uint16_t p, int& r, int& g, int& b) {
    r = ((p >> 11) & 0x1F) * 255 / 31;
    g = ((p >> 5)  & 0x3F) * 255 / 63;
    b = ( p        & 0x1F) * 255 / 31;
}
static int luma_from_rgb(int r, int g, int b) { return (77 * r + 150 * g + 29 * b) >> 8; }

static inline int pixel_luma(const camera_fb_t* fb, int idx) {
    if (fb->format == PIXFORMAT_GRAYSCALE) return fb->buf[idx];
    int r, g, b; rgb565_split(rgb565_at(fb->buf, idx), r, g, b);
    return luma_from_rgb(r, g, b);
}
#if BOLA_MODO_COR
static int max3(int a, int b, int c) { int m = a > b ? a : b; return m > c ? m : c; }
static int min3(int a, int b, int c) { int m = a < b ? a : b; return m < c ? m : c; }
static inline int pixel_chroma(const camera_fb_t* fb, int idx) {
    if (fb->format == PIXFORMAT_GRAYSCALE) return 0;
    int r, g, b; rgb565_split(rgb565_at(fb->buf, idx), r, g, b);
    return max3(r, g, b) - min3(r, g, b);
}
#endif

// ============================================================
//  Decode JPEG -> RGB565 (esta versao do esp32-camera so expoe jpg2rgb565)
// ============================================================
static bool decodifica_jpeg(const uint8_t* src, size_t len) {
    if (!jpg2rgb565(src, len, s_rgb565, JPG_SCALE_NONE)) return false;

    // jpg2rgb565 usa esp_jpeg com swap_color_bytes=0. No ESP32 o decoder ROM
    // parte de RGB888 e grava RGB565 em little-endian; o resto deste modulo
    // usa RGB565 big-endian, igual ao frame buffer da camera. Sem essa troca,
    // o debug/anotacao em modo JPEG fica com canais embaralhados (rosa/magenta).
    for (int i = 0; i < MAX_PIXELS; i++) {
        uint8_t* p = &s_rgb565[2 * i];
        uint8_t t = p[0];
        p[0] = p[1];
        p[1] = t;
    }
    return true;
}

// ============================================================
//  Referencia local (grade de iluminacao / fundo)
// ============================================================
static inline int grid_ref(int x, int y) {
    if (!s_grid_ok) return BOLA_Y_MIN;
    return s_grid[s_celly[y]][s_cellx[x]];   // sem divisao (LUT)
}
static inline int ref_em(int x, int y, int idx) {
    return s_bg_valido ? s_bg[idx] : grid_ref(x, y);
}

static inline bool pixel_candidato(const camera_fb_t* fb, int x, int y, int idx, int* peso_out = nullptr) {
    int luma = pixel_luma(fb, idx);
    int ref = ref_em(x, y, idx);
    int peso = luma - ref;
    if (peso_out) *peso_out = peso;
    bool cand = (luma >= BOLA_Y_MIN) && (peso >= BOLA_DELTA_REF);
#if BOLA_MODO_COR
    if (cand && pixel_chroma(fb, idx) > BOLA_CHROMA_MAX) cand = false;
#else
    (void)x; (void)y;
#endif
    return cand;
}

// Recalcula a grade de iluminacao amostrando a ROI esparsamente.
static void recalcula_grade(const camera_fb_t* fb) {
    int64_t soma[GRADE_NY][GRADE_NX] = {};
    int cnt[GRADE_NY][GRADE_NX] = {};
    int passo = SCAN_STEP * 2; if (passo < 2) passo = 2;
    int64_t soma_total = 0; int n_total = 0;

    for (int y = s_mb_y0; y <= s_mb_y1; y += passo) {
        int lx0 = s_roi_x0[y], lx1 = s_roi_x1[y];
        if (lx0 > lx1) continue;
        int cy = s_celly[y];
        for (int x = lx0; x <= lx1; x += passo) {
            int v = pixel_luma(fb, y * CAM_LARGURA + x);
            soma[cy][s_cellx[x]] += v; cnt[cy][s_cellx[x]]++;
            soma_total += v; n_total++;
        }
    }
    int media_global = n_total ? (int)(soma_total / n_total) : BOLA_Y_MIN;
    for (int j = 0; j < GRADE_NY; j++)
        for (int i = 0; i < GRADE_NX; i++)
            s_grid[j][i] = cnt[j][i] ? (int)(soma[j][i] / cnt[j][i]) : media_global;
    s_grid_ok = true;
    s_media_mesa = media_global;
}

// ============================================================
//  Union-find
// ============================================================
static inline int uf_find(int i) {
    while (s_runs[i].parent != i) { s_runs[i].parent = s_runs[s_runs[i].parent].parent; i = s_runs[i].parent; }
    return i;
}
static inline void uf_union(int a, int b) {
    int ra = uf_find(a), rb = uf_find(b);
    if (ra != rb) s_runs[rb].parent = ra;
}

// ============================================================
//  Desenho (debug)
// ============================================================
static void put_px(uint8_t* buf, int x, int y, uint16_t cor) {
    if (x < 0 || x >= CAM_LARGURA || y < 0 || y >= CAM_ALTURA) return;
    int idx = y * CAM_LARGURA + x;
    if (s_frame_grayscale) {
        int r, g, b; rgb565_split(cor, r, g, b);
        buf[idx] = (uint8_t)luma_from_rgb(r, g, b);
        return;
    }
    buf[2 * idx] = (uint8_t)(cor >> 8);
    buf[2 * idx + 1] = (uint8_t)(cor & 0xFF);
}
static void linha_h(uint8_t* buf, int x0, int x1, int y, uint16_t cor) {
    if (x0 > x1) { int t = x0; x0 = x1; x1 = t; }
    for (int x = x0; x <= x1; x++) put_px(buf, x, y, cor);
}
static void linha_v(uint8_t* buf, int x, int y0, int y1, uint16_t cor) {
    if (y0 > y1) { int t = y0; y0 = y1; y1 = t; }
    for (int y = y0; y <= y1; y++) put_px(buf, x, y, cor);
}
static void linha(uint8_t* buf, int x0, int y0, int x1, int y1, uint16_t cor) {
    int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;
    for (;;) {
        put_px(buf, x0, y0, cor);
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}
static void poligono(uint8_t* buf, const PontoF q[4], uint16_t cor) {
    for (int i = 0; i < 4; i++) {
        int j = (i + 1) & 3;
        linha(buf, (int)lroundf(q[i].x), (int)lroundf(q[i].y),
              (int)lroundf(q[j].x), (int)lroundf(q[j].y), cor);
    }
}
static void retangulo(uint8_t* buf, int x0, int y0, int x1, int y1, uint16_t cor) {
    linha_h(buf, x0, x1, y0, cor); linha_h(buf, x0, x1, y1, cor);
    linha_v(buf, x0, y0, y1, cor); linha_v(buf, x1, y0, y1, cor);
}
static void cruz(uint8_t* buf, int x, int y, uint16_t cor) {
    linha_h(buf, x - 4, x + 4, y, cor); linha_v(buf, x, y - 4, y + 4, cor);
}
static void overlay_sobel(uint8_t* buf, const camera_fb_t* fb) {
    uint8_t linha_ant[CAM_LARGURA];
    uint8_t linha_at[CAM_LARGURA];
    
    // Inicializa a linha anterior (y = 0)
    for (int x = 0; x < CAM_LARGURA; x++) {
        linha_ant[x] = pixel_luma(fb, x);
    }
    
    for (int y = 1; y < CAM_ALTURA - 1; y++) {
        // Copia a linha atual original antes de modificá-la
        for (int x = 0; x < CAM_LARGURA; x++) {
            linha_at[x] = pixel_luma(fb, y * CAM_LARGURA + x);
        }
        
        for (int x = 1; x < CAM_LARGURA - 1; x++) {
            int i = y * CAM_LARGURA + x;
            
            // Linha y-1 (linha anterior)
            int luma_tl = linha_ant[x - 1];
            int luma_tc = linha_ant[x];
            int luma_tr = linha_ant[x + 1];
            
            // Linha y (linha atual)
            int luma_l = linha_at[x - 1];
            int luma_r = linha_at[x + 1];
            
            // Linha y+1 (linha seguinte - ainda não modificada no buffer)
            int luma_bl = pixel_luma(fb, i + CAM_LARGURA - 1);
            int luma_bc = pixel_luma(fb, i + CAM_LARGURA);
            int luma_br = pixel_luma(fb, i + CAM_LARGURA + 1);
            
            int gx = -luma_tl + luma_tr - 2 * luma_l + 2 * luma_r - luma_bl + luma_br;
            int gy = -luma_tl - 2 * luma_tc - luma_tr + luma_bl + 2 * luma_bc + luma_br;
            
            if (abs(gx) + abs(gy) >= BORDA_GRAD_MIN * 8) {
                put_px(buf, x, y, 0xFFFF);
            }
        }
        
        // Copia a linha atual original para ser a anterior na próxima iteração
        memcpy(linha_ant, linha_at, CAM_LARGURA);
    }
}

// ---------- base64 (debug pela serial) ----------
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
                    vTaskDelay(pdMS_TO_TICKS(10)); // cede a CPU temporariamente para a IDLE task resetar o watchdog
                }
            }
        }
    }
    void finish() {
        if (n == 1) b64_triplet(trio[0], 0, 0, 1);
        else if (n == 2) b64_triplet(trio[0], trio[1], 0, 2);
        if (n || col) printf("\n");
        n = 0; col = 0; linhas = 0;
        vTaskDelay(pdMS_TO_TICKS(5)); // Garante yield ao concluir
    }
private:
    uint8_t trio[3] = {}; int n = 0; int col = 0; int linhas = 0;
};

static void envia_ppm_base64(const camera_fb_t* fb, const VisaoDebugInfo& d) {
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
        int r, g, b;
        if (fb->format == PIXFORMAT_GRAYSCALE) { r = g = b = fb->buf[i]; }
        else rgb565_split(rgb565_at(fb->buf, i), r, g, b);
        b64.put((uint8_t)r); b64.put((uint8_t)g); b64.put((uint8_t)b);
    }
    b64.finish();
    printf("===DEBUG_VISAO_BASE64_FIM===\n");
}

// ============================================================
//  Alocacao
// ============================================================
static void* aloca(size_t bytes) {
    void* p = heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!p) p = heap_caps_malloc(bytes, MALLOC_CAP_8BIT);
    return p;
}
static bool aloca_buffers() {
    if (!s_bg)     s_bg     = (uint8_t*)aloca(MAX_PIXELS);
    if (!s_rgb565) s_rgb565 = (uint8_t*)aloca((size_t)MAX_PIXELS * 2);  // sempre: permite JPEG em runtime
    if (!s_runs)   s_runs   = (RunAcc*)aloca(sizeof(RunAcc) * VISAO_MAX_RUNS);
    if (!s_bg || !s_rgb565 || !s_runs) {
        ESP_LOGE(TAG, "sem memoria: bg=%p rgb565=%p runs=%p", s_bg, s_rgb565, s_runs);
        return false;
    }
    ESP_LOGI(TAG, "buffers OK: fundo+rgb565=%.1fKB runs=%.1fKB | scan_step=%d",
             MAX_PIXELS * 3 / 1024.0f, sizeof(RunAcc) * VISAO_MAX_RUNS / 1024.0f, SCAN_STEP);
    return true;
}

// ============================================================
//  NVS (calibracao da camera)
// ============================================================
static void nvs_garante_init() {
    static bool feito = false;
    if (feito) return;
    esp_err_t r = nvs_flash_init();
    if (r == ESP_ERR_NVS_NO_FREE_PAGES || r == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase(); nvs_flash_init();
    }
    feito = true;
}
#if !CAM_AUTO_AJUSTE
static bool nvs_le_cam(int& aec, int& agc) {
#if CAM_USA_NVS
    nvs_garante_init();
    nvs_handle_t h;
    if (nvs_open("visao", NVS_READONLY, &h) != ESP_OK) return false;
    int32_t a = 0, g = 0;
    bool ok = nvs_get_i32(h, "aec", &a) == ESP_OK && nvs_get_i32(h, "agc", &g) == ESP_OK;
    nvs_close(h);
    if (ok) { aec = a; agc = g; }
    return ok;
#else
    (void)aec; (void)agc; return false;
#endif
}
#endif
static void nvs_grava_cam(int aec, int agc) {
#if CAM_USA_NVS
    nvs_garante_init();
    nvs_handle_t h;
    if (nvs_open("visao", NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_i32(h, "aec", aec); nvs_set_i32(h, "agc", agc);
    nvs_commit(h); nvs_close(h);
#else
    (void)aec; (void)agc;
#endif
}

static bool nvs_le_centro(float& dx, float& dy) {
#if CAM_USA_NVS
    nvs_garante_init();
    nvs_handle_t h;
    if (nvs_open("visao", NVS_READONLY, &h) != ESP_OK) return false;
    int32_t x = 0, y = 0;
    bool ok = nvs_get_i32(h, "ctx10", &x) == ESP_OK &&
              nvs_get_i32(h, "cty10", &y) == ESP_OK;
    nvs_close(h);
    if (ok) { dx = x / 10.0f; dy = y / 10.0f; }
    return ok;
#else
    (void)dx; (void)dy; return false;
#endif
}

static void nvs_grava_centro(float dx, float dy) {
#if CAM_USA_NVS
    nvs_garante_init();
    nvs_handle_t h;
    if (nvs_open("visao", NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_i32(h, "ctx10", (int32_t)lroundf(dx * 10.0f));
    nvs_set_i32(h, "cty10", (int32_t)lroundf(dy * 10.0f));
    nvs_commit(h);
    nvs_close(h);
#else
    (void)dx; (void)dy;
#endif
}

static void aplica_sensor_fixo(int aec, int agc) {
    sensor_t* s = esp_camera_sensor_get();
    if (!s) return;
    s->set_brightness(s, 0);
    s->set_contrast(s, 0);
    s->set_saturation(s, 0);
    s->set_whitebal(s, 0);
    s->set_awb_gain(s, 0);
    s->set_exposure_ctrl(s, 0);
    s->set_aec2(s, 0);
    s->set_aec_value(s, aec);
    s->set_gain_ctrl(s, 0);
    s->set_agc_gain(s, agc);
    s->set_lenc(s, 1);
    ESP_LOGI(TAG, "sensor FIXO: aec=%d agc=%d", aec, agc);
}

static void aplica_sensor_auto() {
    sensor_t* s = esp_camera_sensor_get();
    if (!s) return;
    s->set_brightness(s, 1);
    s->set_contrast(s, 1);
    s->set_saturation(s, 0);
    s->set_whitebal(s, 1);
    s->set_awb_gain(s, 1);
    s->set_exposure_ctrl(s, 1);
    s->set_aec2(s, 1);
    s->set_gain_ctrl(s, 1);
    s->set_lenc(s, 1);
    ESP_LOGI(TAG, "sensor AUTO: AEC/AGC/AWB ligados");
}

// ============================================================
//  Inicializacao
// ============================================================
// Inicializa o hardware da camera com o formato do modo atual (s_modo_jpeg).
static esp_err_t inicia_camera_hw() {
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

#if ETAPA == 1
    cfg.pixel_format = PIXFORMAT_JPEG;
    cfg.frame_size   = CAM_STREAM_FRAMESIZE;
    cfg.jpeg_quality = CAM_JPEG_QUALITY;
#else
    cfg.frame_size = CAM_DETECT_FRAMESIZE;
    if (s_modo_jpeg) {
        cfg.pixel_format = PIXFORMAT_JPEG;       // captura comprimida (clock cheio)
        cfg.jpeg_quality = CAM_DETECT_JPEG_Q;
    } else {
  #if BOLA_MODO_COR
        cfg.pixel_format = PIXFORMAT_RGB565;
  #else
        cfg.pixel_format = PIXFORMAT_GRAYSCALE;
  #endif
        cfg.jpeg_quality = 0;
    }
#endif
    return esp_camera_init(&cfg);
}

// (Re)aplica o estado do sensor (exposicao/ganho) apos um (re)init.
void Visao::aplicaEstadoSensor() {
#if CAM_AUTO_AJUSTE
    aplica_sensor_auto();
    autoOn = true;
#else
    if (autoOn) {
        aplica_sensor_auto();
    } else {
        aplica_sensor_fixo(curAec, curAgc);
    }
#endif
}

bool Visao::inicia() {
    prepara_calibracao_mesa();
    if (nvs_le_centro(s_centro_dx, s_centro_dy)) {
        ESP_LOGI(TAG, "centro manual lido da NVS: dx=%.1f dy=%.1f px", s_centro_dx, s_centro_dy);
    }
    prepara_homografia();
    prepara_luts();
    if (!aloca_buffers()) return false;
    mascaraLigada = (OVR_MASCARA_PADRAO != 0);

    esp_err_t err = inicia_camera_hw();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_camera_init falhou: 0x%x (confira pinos e cabo flat)", err);
        return false;
    }

#if !CAM_AUTO_AJUSTE
    {
        int aec = CAM_AEC_FIXO, agc = CAM_AGC_FIXO;
        if (nvs_le_cam(aec, agc)) ESP_LOGI(TAG, "calibracao da camera lida da NVS");
        curAec = aec; curAgc = agc; autoOn = false;
    }
#endif
    aplicaEstadoSensor();

    ESP_LOGI(TAG, "Camera OV2640 iniciada (modo=%s) | homografia=%s",
             s_modo_jpeg ? "JPEG" : (BOLA_MODO_COR ? "RGB565" : "GRAYSCALE"),
             s_H_ok ? "OK" : "FALHOU");
    return true;
}

bool Visao::alternaCaptura() {
    s_modo_jpeg = !s_modo_jpeg;
    esp_camera_deinit();
    esp_err_t err = inicia_camera_hw();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "reinit da camera falhou: 0x%x — revertendo", err);
        s_modo_jpeg = !s_modo_jpeg;
        esp_camera_deinit();
        inicia_camera_hw();
    }
    aplicaEstadoSensor();
    s_grid_ok = false; // s_bg_valido = false;   // referencia depende do frame; recalibra
    resetFiltro();
    ESP_LOGI(TAG, "modo de captura -> %s", s_modo_jpeg ? "JPEG" : "GRAYSCALE/RGB565");
    return s_modo_jpeg;
}

bool Visao::modoJpeg() const { return s_modo_jpeg; }

// ============================================================
//  Filtro alfa-beta
// ============================================================
static inline void ab_predict(float x, float y, float vx, float vy, float dt, float& px, float& py) {
    px = x + vx * dt; py = y + vy * dt;
}

// ============================================================
//  Deteccao
// ============================================================
Medicao Visao::detecta() {
    Medicao m = {};
    m.t_us = esp_timer_get_time();
    int64_t t0 = m.t_us;

    camera_fb_t* raw = esp_camera_fb_get();
    int64_t t_cap = esp_timer_get_time();
    if (!raw) { ESP_LOGW(TAG, "falha ao capturar frame"); return m; }

    // frame de trabalho: o raw, ou a matriz RGB565 decodificada do JPEG
    const camera_fb_t* fb = raw;
    camera_fb_t deco = {};
    if (raw->format == PIXFORMAT_JPEG) {
        if (!decodifica_jpeg(raw->buf, raw->len)) {
            ESP_LOGW(TAG, "falha ao decodificar JPEG");
            esp_camera_fb_return(raw);
            return m;
        }
        deco.buf = s_rgb565; deco.len = (size_t)MAX_PIXELS * 2;
        deco.width = CAM_LARGURA; deco.height = CAM_ALTURA; deco.format = PIXFORMAT_RGB565;
        esp_camera_fb_return(raw); raw = NULL;   // libera o buffer da camera ja
        fb = &deco;
    }

    s_frame_grayscale = (fb->format == PIXFORMAT_GRAYSCALE);
    if (fb->width != CAM_LARGURA || fb->height != CAM_ALTURA) {
        ESP_LOGE(TAG, "frame inesperado: %dx%d fmt=%d", (int)fb->width, (int)fb->height, fb->format);
        if (raw) esp_camera_fb_return(raw);
        return m;
    }

    float dt = (ultimoT > 0) ? (t0 - ultimoT) / 1e6f : PID_DT;
    if (dt < 0.001f) dt = 0.001f;
    if (dt > 0.2f)   dt = 0.2f;
    ultimoT = t0;
    m.dt = dt;

    bool mandar_debug = debugSolicitado;

    // ---------- referencia local (grade), periodica ----------
    if (!s_bg_valido && (!s_grid_ok || framesAteMedia <= 0 || mandar_debug)) {
        recalcula_grade(fb);
        framesAteMedia = MEDIA_INTERVALO;
    } else if (!s_bg_valido) {
        framesAteMedia--;
    }
    mediaMesaY = s_bg_valido ? -1 : s_media_mesa;

    // ---------- janela de varredura (tracking) ----------
#if VISAO_ROI_LIVRE || !VISAO_JANELA_DINAMICA
    bool tracking = false;   // janela dinamica desligada: varre o ROI todo sempre
#else
    bool tracking = framesPerdidos < ROI_PERDE_FRAMES && ultimoCx > 0 && ultimoCy > 0;
#endif
    int sx0 = s_mb_x0, sy0 = s_mb_y0, sx1 = s_mb_x1, sy1 = s_mb_y1;
    if (tracking) {
        int ww = (int)(CAM_LARGURA * ROI_FRACAO); if (ww < 24) ww = 24;
        int wh = (int)(CAM_ALTURA  * ROI_FRACAO); if (wh < 18) wh = 18;
        sx0 = clamp_int(ultimoCx - ww / 2, s_mb_x0, s_mb_x1);
        sx1 = clamp_int(ultimoCx + ww / 2, s_mb_x0, s_mb_x1);
        sy0 = clamp_int(ultimoCy - wh / 2, s_mb_y0, s_mb_y1);
        sy1 = clamp_int(ultimoCy + wh / 2, s_mb_y0, s_mb_y1);
    }

    // ---------- componentes conexos (run-length + union-find) ----------
    int n = 0;
    int prevStart = 0, prevCount = 0;
    bool overflow = false;
    const int S = SCAN_STEP;

    for (int y = sy0; y <= sy1 && !overflow; y += S) {
        int lx0 = s_roi_x0[y], lx1 = s_roi_x1[y];
        if (lx0 > lx1) { prevStart = n; prevCount = 0; continue; }
        if (lx0 < sx0) lx0 = sx0;
        if (lx1 > sx1) lx1 = sx1;
        if (lx0 > lx1) { prevStart = n; prevCount = 0; continue; }

        int curStart = n;
        int x = lx0;
        while (x <= lx1) {
            int idx = y * CAM_LARGURA + x;
            int peso = 0;
            bool cand = pixel_candidato(fb, x, y, idx, &peso);
            if (!cand) { x += S; continue; }

            int rx0 = x;
            int64_t sxa = 0, sya = 0, wsa = 0; int area = 0;
            while (x <= lx1) {
                idx = y * CAM_LARGURA + x;
                cand = pixel_candidato(fb, x, y, idx, &peso);
                if (!cand) break;
#if CENTROIDE_PONDERADO
                int w = peso > 1 ? peso : 1;
#else
                int w = 1;
#endif
                area++; sxa += (int64_t)x * w; sya += (int64_t)y * w; wsa += w;
                x += S;
            }
            int rx1 = x - S;

            RunAcc& r = s_runs[n];
            r.x0 = rx0; r.x1 = rx1;
            r.bx0 = rx0; r.bx1 = rx1; r.by0 = y; r.by1 = y;
            r.area = area; r.sx = sxa; r.sy = sya; r.wsum = wsa;
            r.parent = n;

            for (int k = prevStart; k < prevStart + prevCount; k++) {
                if (r.x0 <= s_runs[k].x1 + S && s_runs[k].x0 <= r.x1 + S) uf_union(n, k);
            }
            n++;
            if (n >= VISAO_MAX_RUNS) { overflow = true; break; }
        }
        prevStart = curStart; prevCount = n - curStart;
    }

    // ---------- consolida runs em blobs (por raiz) ----------
    for (int i = 0; i < n; i++) {
        int root = uf_find(i);
        if (root == i) continue;
        RunAcc& a = s_runs[root];
        RunAcc& b = s_runs[i];
        a.area += b.area; a.sx += b.sx; a.sy += b.sy; a.wsum += b.wsum;
        if (b.bx0 < a.bx0) a.bx0 = b.bx0;
        if (b.bx1 > a.bx1) a.bx1 = b.bx1;
        if (b.by0 < a.by0) a.by0 = b.by0;
        if (b.by1 > a.by1) a.by1 = b.by1;
    }

    // ---------- escolhe o melhor blob por score ----------
    int melhor = -1; float melhor_score = -1.0f; int blobs = 0;
    int area_min_alvo = BOLA_AREA_ALVO * (100 - BOLA_AREA_TOL_PCT) / 100;
    int area_max_alvo = BOLA_AREA_ALVO * (100 + BOLA_AREA_TOL_PCT) / 100;
    if (area_min_alvo < BOLA_MIN_PIXELS) area_min_alvo = BOLA_MIN_PIXELS;
    if (area_max_alvo > BOLA_MAX_PIXELS) area_max_alvo = BOLA_MAX_PIXELS;
    for (int i = 0; i < n; i++) {
        if (uf_find(i) != i) continue;
        RunAcc& r = s_runs[i];
        int w = r.bx1 - r.bx0 + 1;
        int h = r.by1 - r.by0 + 1;
        int area_est = r.area * S * S;
        bool tamanho = area_est >= area_min_alvo && area_est <= area_max_alvo;
        bool lados   = w >= BOLA_MIN_LADO && h >= BOLA_MIN_LADO && w <= BOLA_MAX_LADO && h <= BOLA_MAX_LADO;
        int menor_lado = w < h ? w : h;
        int maior_lado = w > h ? w : h;
        int aspecto_pct = maior_lado > 0 ? (menor_lado * 100) / maior_lado : 0;
        bool forma = aspecto_pct >= BOLA_ASPECT_MIN_PCT;
        int bbox_area = w * h;
        int densidade_pct = bbox_area > 0 ? (area_est * 100) / bbox_area : 0;
        bool densidade = densidade_pct >= BOLA_DENSIDADE_MIN_PCT;
        if (!(tamanho && lados && forma && densidade)) continue;
        blobs++;

        int area_err_pct = BOLA_AREA_ALVO > 0 ? abs(area_est - BOLA_AREA_ALVO) * 100 / BOLA_AREA_ALVO : 100;
        if (area_err_pct > 100) area_err_pct = 100;
        int area_score = 100 - area_err_pct;
        int contraste_medio = r.area > 0 ? (int)(r.wsum / r.area) : 0;
        float score = (float)area_est;
        score += area_score * 8.0f;
        score += aspecto_pct * 4.0f;
        score += densidade_pct * 2.0f;
        score += contraste_medio * 2.0f;
        if (score > melhor_score) { melhor_score = score; melhor = i; }
    }

    // ---------- resultado cru ----------
    float cx_f = 0, cy_f = 0; bool achou = false; int area_blob = 0;
    int bx0 = 0, by0 = 0, bx1 = 0, by1 = 0;
    if (melhor >= 0) {
        RunAcc& r = s_runs[melhor];
        cx_f = (float)r.sx / (r.wsum ? r.wsum : 1);
        cy_f = (float)r.sy / (r.wsum ? r.wsum : 1);
        area_blob = r.area * S * S;
        bx0 = r.bx0; by0 = r.by0; bx1 = r.bx1; by1 = r.by1;

#if REFINO_SUBPIXEL
        if (S > 1) {
            int64_t rsx = 0, rsy = 0, rws = 0;
            for (int y = by0; y <= by1; y++) {
                int lx0 = s_roi_x0[y], lx1 = s_roi_x1[y];
                int a = bx0 > lx0 ? bx0 : lx0;
                int b = bx1 < lx1 ? bx1 : lx1;
                for (int x = a; x <= b; x++) {
                    int idx = y * CAM_LARGURA + x;
                    int peso = 0;
                    if (!pixel_candidato(fb, x, y, idx, &peso)) continue;
                    int wp = peso > 1 ? peso : 1;
                    rsx += (int64_t)x * wp; rsy += (int64_t)y * wp; rws += wp;
                }
            }
            if (rws > 0) { cx_f = (float)rsx / rws; cy_f = (float)rsy / rws; }
        }
#endif
        achou = true;
    }

    // ---------- pixel -> cm ----------
    float raw_x = 0, raw_y = 0;
    if (achou) pixel_para_cm(cx_f, cy_f, raw_x, raw_y);

    // ---------- gating + filtro alfa-beta ----------
    float out_x = raw_x, out_y = raw_y, out_vx = 0, out_vy = 0;
    bool gated = false;
#if FILTRO_ATIVO
    if (achou) {
        if (filtroIniciado) {
            float px, py; ab_predict(fx, fy, fvx, fvy, dt, px, py);
#if FILTRO_GATING_ATIVO
            float dmax = FILTRO_VEL_MAX_CM_S * dt + 2.0f;
            if (distf(raw_x, raw_y, px, py) > dmax) {
                gated = true; achou = false;
            } else {
                float rx = raw_x - px, ry = raw_y - py;
                fx = px + FILTRO_ALFA * rx; fy = py + FILTRO_ALFA * ry;
                fvx += FILTRO_BETA * rx / dt; fvy += FILTRO_BETA * ry / dt;
            }
#else
            float rx = raw_x - px, ry = raw_y - py;
            fx = px + FILTRO_ALFA * rx; fy = py + FILTRO_ALFA * ry;
            fvx += FILTRO_BETA * rx / dt; fvy += FILTRO_BETA * ry / dt;
#endif
        } else {
            fx = raw_x; fy = raw_y; fvx = 0; fvy = 0; filtroIniciado = true;
        }
        out_x = fx; out_y = fy; out_vx = fvx; out_vy = fvy;
    }
#endif

    // ---------- atualiza tracking ----------
    if (achou) {
        ultimoX = out_x; ultimoY = out_y;
        ultimoCx = (int)lroundf(cx_f); ultimoCy = (int)lroundf(cy_f);
        framesPerdidos = 0;
        m.x = out_x; m.y = out_y; m.vx = out_vx; m.vy = out_vy; m.achou = true;
    } else {
        framesPerdidos++;
        if (framesPerdidos > ROI_PERDE_FRAMES) filtroIniciado = false;
    }

    // ---------- atualizacao lenta do fundo (fora da bola) ----------
#if FUNDO_ATUALIZA
    if (s_bg_valido) {
        int passo = SCAN_STEP * 2; if (passo < 2) passo = 2;
        for (int y = s_mb_y0; y <= s_mb_y1; y += passo) {
            int lx0 = s_roi_x0[y], lx1 = s_roi_x1[y];
            if (lx0 > lx1) continue;
            bool linha_bola = achou && y >= by0 - 2 && y <= by1 + 2;
            for (int x = lx0; x <= lx1; x += passo) {
                if (linha_bola && x >= bx0 - 2 && x <= bx1 + 2) continue;
                int idx = y * CAM_LARGURA + x;
                int luma = pixel_luma(fb, idx);
                s_bg[idx] += (luma - s_bg[idx]) >> FUNDO_ALPHA_SHIFT;
            }
        }
    }
#endif

    // ---------- debug info ----------
    dbg.achou = m.achou;
    dbg.cx = (int)lroundf(cx_f); dbg.cy = (int)lroundf(cy_f);
    dbg.cx_f = cx_f; dbg.cy_f = cy_f;
    dbg.area = achou ? area_blob : 0;
    dbg.x0 = bx0; dbg.y0 = by0; dbg.x1 = bx1; dbg.y1 = by1;
    dbg.roi_x0 = sx0; dbg.roi_y0 = sy0; dbg.roi_x1 = sx1; dbg.roi_y1 = sy1;
    dbg.media_y = mediaMesaY; dbg.blobs = blobs; dbg.runs = n; dbg.overflow = overflow;
    dbg.usou_fundo = s_bg_valido;
    dbg.mesa_x_cm = raw_x; dbg.mesa_y_cm = raw_y;
    dbg.filt_x_cm = out_x; dbg.filt_y_cm = out_y;
    dbg.vel_x_cm = out_vx; dbg.vel_y_cm = out_vy;
    dbg.captura_us = t_cap - t0;
    dbg.dt_us = esp_timer_get_time() - t0;
    dbg.processo_us = dbg.dt_us - dbg.captura_us;
    (void)gated;

    // ---------- debug visual (imagem ORIGINAL + overlays) ----------
    if (mandar_debug) {
        uint8_t* buf = fb->buf;
        if (mascaraLigada) {
            for (int y = 0; y < CAM_ALTURA; y++) {
                int lx0 = 1, lx1 = 0;
                if (y >= sy0 && y <= sy1) {
                    lx0 = s_roi_x0[y];
                    lx1 = s_roi_x1[y];
                    if (lx0 < sx0) lx0 = sx0;
                    if (lx1 > sx1) lx1 = sx1;
                }
                for (int x = 0; x < CAM_LARGURA; x++) {
                    bool cand = (lx0 <= lx1 && x >= lx0 && x <= lx1) &&
                                pixel_candidato(fb, x, y, y * CAM_LARGURA + x);
                    put_px(buf, x, y, cand ? 0xFFFF : 0x0000);
                }
            }
        } else if (sobelLigado) {
            overlay_sobel(buf, fb);
        }
#if OVR_MESA
        poligono(buf, s_mesa_ext, 0xFFE0);
#endif
#if OVR_ROI
        poligono(buf, s_mesa_roi, 0x07E0);
#endif
#if OVR_JANELA
        retangulo(buf, sx0, sy0, sx1, sy1, 0xF81F);
#endif
#if OVR_EIXOS
        { PontoF c = centro_origem_px();
          cruz(buf, (int)lroundf(c.x), (int)lroundf(c.y), 0xFFE0); }
#endif
        if (melhor >= 0) {
#if OVR_CANDIDATOS
            for (int y = by0; y <= by1; y++) {
                int lx0 = s_roi_x0[y], lx1 = s_roi_x1[y];
                int a = bx0 > lx0 ? bx0 : lx0, b = bx1 < lx1 ? bx1 : lx1;
                for (int x = a; x <= b; x++) {
                    int idx = y * CAM_LARGURA + x;
                    if (pixel_candidato(fb, x, y, idx))
                        put_px(buf, x, y, 0x07FF);
                }
            }
#endif
#if OVR_CAIXA
            retangulo(buf, bx0, by0, bx1, by1, 0xF800);
#endif
#if OVR_CENTROIDE
            cruz(buf, dbg.cx, dbg.cy, 0x001F);
#endif
        }
        envia_ppm_base64(fb, dbg);
        debugSolicitado = false;
    }

    if (raw) esp_camera_fb_return(raw);
    return m;
}

// ============================================================
//  Metodos publicos
// ============================================================
void Visao::calibraCor() { capturaFundo(); }

void Visao::solicitaDebug() { debugSolicitado = true; }

void Visao::ajustaCentroPx(float dx, float dy) {
    s_centro_dx += dx;
    s_centro_dy += dy;
    resetFiltro();
    printf("Centro manual: dx=%.1f dy=%.1f px\n", s_centro_dx, s_centro_dy);
}

void Visao::ajustaCentroCm(float dx, float dy) {
    PontoF c = centro_mesa_px();
    float x0 = 0, y0 = 0, x1 = 0, y1 = 0;

    if (dx != 0.0f) {
        homografia_px_para_cm(c.x, c.y, x0, y0);
        homografia_px_para_cm(c.x + 1.0f, c.y, x1, y1);
        float cm_por_px = x1 - x0;
        if (fabsf(cm_por_px) > 1e-4f) s_centro_dx += dx / cm_por_px;
    }

    if (dy != 0.0f) {
        homografia_px_para_cm(c.x, c.y, x0, y0);
        homografia_px_para_cm(c.x, c.y + 1.0f, x1, y1);
        float cm_por_px = y1 - y0;
        if (fabsf(cm_por_px) > 1e-4f) s_centro_dy += dy / cm_por_px;
    }

    resetFiltro();
    printf("Centro manual: offset=(%.1f, %.1f) px  ajuste_cm=(%.2f, %.2f)\n",
           s_centro_dx, s_centro_dy, dx, dy);
}

void Visao::salvaCentroPx() {
    nvs_grava_centro(s_centro_dx, s_centro_dy);
    printf("Centro manual salvo: dx=%.1f dy=%.1f px\n", s_centro_dx, s_centro_dy);
}

void Visao::resetCentroPx() {
    s_centro_dx = 0.0f;
    s_centro_dy = 0.0f;
    resetFiltro();
    printf("Centro manual voltou ao padrao: dx=0.0 dy=0.0 px\n");
}

void Visao::imprimeCentroPx() {
    PontoF c = centro_origem_px();
    printf("Centro/origem: px=(%.1f, %.1f) offset=(%.1f, %.1f)\n",
           c.x, c.y, s_centro_dx, s_centro_dy);
}

void Visao::capturaFundo() {
    camera_fb_t* raw = esp_camera_fb_get();
    if (!raw) { ESP_LOGW(TAG, "capturaFundo: sem frame"); return; }
    const camera_fb_t* fb = raw;
    camera_fb_t deco = {};
    if (raw->format == PIXFORMAT_JPEG) {
        if (!decodifica_jpeg(raw->buf, raw->len)) {
            esp_camera_fb_return(raw);
            ESP_LOGW(TAG, "capturaFundo: decode falhou");
            return;
        }
        deco.buf = s_rgb565; deco.width = CAM_LARGURA; deco.height = CAM_ALTURA; deco.format = PIXFORMAT_RGB565;
        esp_camera_fb_return(raw); raw = NULL; fb = &deco;
    }
    s_frame_grayscale = (fb->format == PIXFORMAT_GRAYSCALE);
    if (fb->width != CAM_LARGURA || fb->height != CAM_ALTURA) {
        if (raw) esp_camera_fb_return(raw);
        ESP_LOGW(TAG, "capturaFundo: frame invalido");
        return;
    }
    for (int i = 0; i < MAX_PIXELS; i++) s_bg[i] = pixel_luma(fb, i);
    s_bg_valido = true;
    if (raw) esp_camera_fb_return(raw);
    ESP_LOGI(TAG, "fundo da mesa capturado (subtracao de fundo ATIVA)");
}

void Visao::limpaFundo() {
    s_bg_valido = false; s_grid_ok = false;
    ESP_LOGI(TAG, "fundo descartado (volta para grade de iluminacao)");
}

void Visao::calibraCamera() {
    sensor_t* s = esp_camera_sensor_get();
    if (!s) return;
    ESP_LOGI(TAG, "calibrando camera: auto por %d ms...", CAM_CALIBRA_MS);
    aplica_sensor_auto();
    int passos = CAM_CALIBRA_MS / 50; if (passos < 1) passos = 1;
    for (int i = 0; i < passos; i++) {
        camera_fb_t* fb = esp_camera_fb_get();
        if (fb) esp_camera_fb_return(fb);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    int aec = s->status.aec_value;
    int agc = s->status.agc_gain;
    if (aec <= 0) aec = CAM_AEC_FIXO;
    curAec = aec; curAgc = agc; autoOn = false;
    aplica_sensor_fixo(aec, agc);
    nvs_grava_cam(aec, agc);
    s_grid_ok = false; s_bg_valido = false;
    ESP_LOGI(TAG, "camera calibrada e salva na NVS: aec=%d agc=%d", aec, agc);
}

void Visao::resetFiltro() {
    filtroIniciado = false; fvx = fvy = 0; framesPerdidos = ROI_PERDE_FRAMES;
    ESP_LOGI(TAG, "filtro alfa-beta reiniciado");
}

void Visao::debugSobel(bool on) { sobelLigado = on; }

void Visao::debugMascara(bool on) { mascaraLigada = on; }

// --- Detecção automática de bordas e calibração geométrica da mesa ---

static inline int sobel_at(const camera_fb_t* fb, int x, int y) {
    int i = y * CAM_LARGURA + x;
    int gx = -pixel_luma(fb, i - CAM_LARGURA - 1) + pixel_luma(fb, i - CAM_LARGURA + 1)
             - 2 * pixel_luma(fb, i - 1) + 2 * pixel_luma(fb, i + 1)
             - pixel_luma(fb, i + CAM_LARGURA - 1) + pixel_luma(fb, i + CAM_LARGURA + 1);
    int gy = -pixel_luma(fb, i - CAM_LARGURA - 1) - 2 * pixel_luma(fb, i - CAM_LARGURA) - pixel_luma(fb, i - CAM_LARGURA + 1)
             + pixel_luma(fb, i + CAM_LARGURA - 1) + 2 * pixel_luma(fb, i + CAM_LARGURA) + pixel_luma(fb, i + CAM_LARGURA + 1);
    return abs(gx) + abs(gy);
}

static PontoF detecta_borda_na_direcao(const camera_fb_t* fb, float dx, float dy) {
    float x = CENTRO_X;
    float y = CENTRO_Y;
    float passo = 0.5f; // passo de varredura fina
    
    // Calcula a luma média da mesa perto do centro (primeiros 15 passos, ~7.5 pixels)
    float luma_mesa_sum = 0.0f;
    for (int i = 0; i < 15; i++) {
        int px = (int)lroundf(x + dx * passo * i);
        int py = (int)lroundf(y + dy * passo * i);
        luma_mesa_sum += pixel_luma(fb, py * CAM_LARGURA + px);
    }
    float luma_mesa = luma_mesa_sum / 15.0f;

    int melhor_x = -1, melhor_y = -1;
    int max_grad = 0;
    int px_fim = -1, py_fim = -1;
    
    // Varre até bater no limite da tela (margem de segurança de 2 pixels)
    while (x > 1 && x < CAM_LARGURA - 2 && y > 1 && y < CAM_ALTURA - 2) {
        int px = (int)lroundf(x);
        int py = (int)lroundf(y);
        int grad = sobel_at(fb, px, py);
        if (grad > max_grad) {
            max_grad = grad;
            melhor_x = px;
            melhor_y = py;
        }
        px_fim = px;
        py_fim = py;
        x += dx * passo;
        y += dy * passo;
    }
    
    // Se o gradiente máximo encontrado for significativo
    if (max_grad >= 30 && melhor_x != -1) {
        // Calcula a luma média na extremidade do escaneamento (últimos 6 passos, ~3 pixels)
        float luma_fim_sum = 0.0f;
        for (int i = 0; i < 6; i++) {
            int px = (int)lroundf(px_fim - dx * passo * i);
            int py = (int)lroundf(py_fim - dy * passo * i);
            luma_fim_sum += pixel_luma(fb, py * CAM_LARGURA + px);
        }
        float luma_fim = luma_fim_sum / 6.0f;

        // Se a luminosidade da extremidade for muito próxima da luminosidade da mesa,
        // significa que não cruzamos nenhuma borda física (o escaneamento terminou ainda dentro do material da mesa)
        // e o gradiente detectado foi apenas um ruído/linha de grade interna.
        float diff = luma_fim - luma_mesa;
        if (diff < 0) diff = -diff;

        ESP_LOGI("BORDA_DBG", "Dir=(%.1f, %.1f) luma_mesa=%.1f max_grad=%d melhor=(%d, %d) luma_fim=%.1f diff=%.1f",
                 dx, dy, luma_mesa, max_grad, melhor_x, melhor_y, luma_fim, diff);

        if (diff < 25.0f) {
            ESP_LOGI("BORDA_DBG", "  -> REJEITADO (diff < 25.0)");
            return { -1.0f, -1.0f }; // Rejeita e trata como borda não encontrada
        }

        return { (float)melhor_x, (float)melhor_y };
    }
    ESP_LOGI("BORDA_DBG", "Dir=(%.1f, %.1f) -> REJEITADO (max_grad %d < 30)", dx, dy, max_grad);
    return { -1.0f, -1.0f }; // Borda não encontrada
}

// Detecta uma lateral da mesa (ESQ: lado=-1, DIR: lado=+1) como uma LINHA
// x = slope*y + intercept, amostrando varias alturas. Ajuste por minimos
// quadrados com 1 passada de rejeicao de outliers (bola/ruido caem fora).
// Retorna o nro de pontos usados (0 = falhou) e preenche slope/intercept +
// o ponto medio (xmid,ymid) da borda.
static int detecta_lateral(const camera_fb_t* fb, int lado,
                           float& slope, float& intercept, float& xmid, float& ymid) {
    const int Y0 = 10, Y1 = CAM_ALTURA - 10, STEP = 2;
    float px[CAM_ALTURA], py[CAM_ALTURA];
    int n = 0;
    for (int y = Y0; y <= Y1; y += STEP) {
        int bestx = -1, bestg = 0;
        for (int x = CENTRO_X; x > 2 && x < CAM_LARGURA - 3; x += lado) {
            int g = sobel_at(fb, x, y);
            if (g > bestg) { bestg = g; bestx = x; }
        }
        if (bestg >= 40 && bestx > 2 && bestx < CAM_LARGURA - 3) {
            px[n] = (float)bestx; py[n] = (float)y; n++;
        }
    }
    if (n < 5) return 0;

    for (int pass = 0; pass < 2; pass++) {
        double sx = 0, sy = 0, sxy = 0, syy = 0; int m = 0;
        for (int i = 0; i < n; i++) {
            if (px[i] < 0) continue;
            sx += px[i]; sy += py[i]; sxy += (double)px[i] * py[i]; syy += (double)py[i] * py[i]; m++;
        }
        if (m < 4) return 0;
        double denom = (double)m * syy - sy * sy;
        if (fabs(denom) < 1e-6) { slope = 0.0f; intercept = (float)(sx / m); }
        else { slope = (float)(((double)m * sxy - sy * sx) / denom); intercept = (float)((sx - slope * sy) / m); }
        ymid = (float)(sy / m); xmid = slope * ymid + intercept;
        if (pass == 0) {                       // rejeita pontos > 4 px da reta
            for (int i = 0; i < n; i++) {
                if (px[i] < 0) continue;
                if (fabsf(px[i] - (slope * py[i] + intercept)) > 4.0f) px[i] = -1.0f;
            }
        }
    }
    int used = 0; for (int i = 0; i < n; i++) if (px[i] >= 0) used++;
    return used;
}

bool Visao::calibraMesaGeometrica() {
    camera_fb_t* raw = esp_camera_fb_get();
    if (!raw) { ESP_LOGW(TAG, "calibraMesa: sem frame"); return false; }
    const camera_fb_t* fb = raw;
    camera_fb_t deco = {};
    if (raw->format == PIXFORMAT_JPEG) {
        if (!decodifica_jpeg(raw->buf, raw->len)) {
            esp_camera_fb_return(raw);
            ESP_LOGW(TAG, "calibraMesa: decode falhou");
            return false;
        }
        deco.buf = s_rgb565; deco.width = CAM_LARGURA; deco.height = CAM_ALTURA; deco.format = PIXFORMAT_RGB565;
        esp_camera_fb_return(raw); raw = NULL; fb = &deco;
    }

    // --- NOVO: detecta as laterais ESQ/DIR como LINHAS, mede a inclinacao real
    //     e monta um quadrado ROTACIONADO. Extrapola topo/baixo (nao aparecem)
    //     como quadrado. So cai no metodo antigo se nao achar as duas laterais.
    {
        float slL, bL, xmL, ymL, slR, bR, xmR, ymR;
        int nL = detecta_lateral(fb, -1, slL, bL, xmL, ymL);
        int nR = detecta_lateral(fb, +1, slR, bR, xmR, ymR);
        if (nL >= 5 && nR >= 5) {
            float sl  = 0.5f * (slL + slR);            // inclinacao media (dx/dy)
            float inv = 1.0f / sqrtf(sl * sl + 1.0f);
            float ex = sl * inv, ey = inv;             // versor "descendo" a borda
            float ymid = 0.5f * (ymL + ymR);
            float xLm = slL * ymid + bL;
            float xRm = slR * ymid + bR;
            float W = (xRm - xLm) * ey;                // lado do quadrado (perpendicular)
            if (W >= 20.0f) {
                float hx = ex * (W * 0.5f), hy = ey * (W * 0.5f);  // meio-lado ao longo da borda
                s_mesa_ext[0] = { xLm - hx, ymid - hy };   // TL
                s_mesa_ext[1] = { xRm - hx, ymid - hy };   // TR
                s_mesa_ext[2] = { xRm + hx, ymid + hy };   // BR
                s_mesa_ext[3] = { xLm + hx, ymid + hy };   // BL
                float cx = CENTRO_X, cy = CENTRO_Y;
                for (int i = 0; i < 4; i++) {
                    s_mesa_roi[i].x = s_mesa_ext[i].x + (cx - s_mesa_ext[i].x) * 0.12f;
                    s_mesa_roi[i].y = s_mesa_ext[i].y + (cy - s_mesa_ext[i].y) * 0.12f;
                }
                prepara_homografia();
                prepara_luts();                        // ATUALIZA a regiao de varredura!
                s_grid_ok = false;
                float ang = atan2f(ex, ey) * 57.29578f;
                ESP_LOGI(TAG, "MESA CALIBRADA (laterais): rot=%.1f deg W=%.0fpx nL=%d nR=%d", ang, W, nL, nR);
                ESP_LOGI(TAG, "  TL=(%.1f,%.1f) TR=(%.1f,%.1f) BR=(%.1f,%.1f) BL=(%.1f,%.1f)",
                         s_mesa_ext[0].x, s_mesa_ext[0].y, s_mesa_ext[1].x, s_mesa_ext[1].y,
                         s_mesa_ext[2].x, s_mesa_ext[2].y, s_mesa_ext[3].x, s_mesa_ext[3].y);
                if (raw) esp_camera_fb_return(raw);
                return true;
            }
        }
        ESP_LOGW(TAG, "calibraMesa: laterais incompletas (nL=%d nR=%d) -> metodo antigo (reto)", nL, nR);
    }

    // Busca as bordas nos eixos principais (ortogonais)
    PontoF pt = detecta_borda_na_direcao(fb, 0.0f, -1.0f); // Topo
    PontoF pb = detecta_borda_na_direcao(fb, 0.0f, 1.0f);  // Baixo
    PontoF pl = detecta_borda_na_direcao(fb, -1.0f, 0.0f); // Esquerda
    PontoF pr = detecta_borda_na_direcao(fb, 1.0f, 0.0f);  // Direita

    if (raw) esp_camera_fb_return(raw);

    // Valida os pontos (garante que não estão colados nos limites extremos do sensor)
    auto valido_x = [](PontoF p) { return p.x > 3 && p.x < CAM_LARGURA - 4; };
    auto valido_y = [](PontoF p) { return p.y > 3 && p.y < CAM_ALTURA - 4; };

    bool v_pt = valido_y(pt);
    bool v_pb = valido_y(pb);
    bool v_pl = valido_x(pl);
    bool v_pr = valido_x(pr);

    // 1. Se as laterais X estão visíveis, usamos a proporção do quadrado da mesa para extrapolar a altura Y cortada
    if (v_pl && v_pr) {
        float largura_px = pr.x - pl.x;
        if (!v_pt && v_pb) { pt.y = pb.y - largura_px; v_pt = true; }
        if (!v_pb && v_pt) { pb.y = pt.y + largura_px; v_pb = true; }
        if (!v_pt && !v_pb) {
            // Se nenhuma borda Y está visível, extrapolamos a altura igual à largura e centralizamos
            pt.y = CENTRO_Y - largura_px * 0.5f;
            pb.y = CENTRO_Y + largura_px * 0.5f;
            v_pt = v_pb = true;
        }
    }
    
    // 2. Se as bordas Y estão visíveis, usamos a proporção do quadrado da mesa para extrapolar a largura X cortada
    if (v_pt && v_pb) {
        float altura_px = pb.y - pt.y;
        if (!v_pl && v_pr) { pl.x = pr.x - altura_px; v_pl = true; }
        if (!v_pr && v_pl) { pr.x = pl.x + altura_px; v_pr = true; }
        if (!v_pl && !v_pr) {
            // Se nenhuma borda X está visível, extrapolamos a largura igual à altura e centralizamos
            pl.x = CENTRO_X - altura_px * 0.5f;
            pr.x = CENTRO_X + altura_px * 0.5f;
            v_pl = v_pr = true;
        }
    }

    // 3. Fallback de simetria (se ainda faltar alguma borda não oposta)
    if (!v_pl && v_pr) { pl.x = 2 * CENTRO_X - pr.x; v_pl = true; }
    if (!v_pr && v_pl) { pr.x = 2 * CENTRO_X - pl.x; v_pr = true; }
    if (!v_pt && v_pb) { pt.y = 2 * CENTRO_Y - pb.y; v_pt = true; }
    if (!v_pb && v_pt) { pb.y = 2 * CENTRO_Y - pt.y; v_pb = true; }

    // 4. Fallback absoluto (se a câmera estiver muito colada e cortar tudo)
    if (!v_pl && !v_pr) {
        pl.x = 8;
        pr.x = CAM_LARGURA - 9;
        v_pl = v_pr = true;
    }
    if (!v_pt && !v_pb) {
        pt.y = 8;
        pb.y = CAM_ALTURA - 9;
        v_pt = v_pb = true;
    }

    // Define os 4 cantos de forma perfeitamente retangular (90 graus)
    s_mesa_ext[0] = { pl.x, pt.y }; // TL
    s_mesa_ext[1] = { pr.x, pt.y }; // TR
    s_mesa_ext[2] = { pr.x, pb.y }; // BR
    s_mesa_ext[3] = { pl.x, pb.y }; // BL

    // Define a ROI interna da mesa proporcionalmente com margem de segurança (12% para dentro)
    float margem = 0.12f;
    float cx = CENTRO_X;
    float cy = CENTRO_Y;
    for (int i = 0; i < 4; i++) {
        s_mesa_roi[i].x = s_mesa_ext[i].x + (cx - s_mesa_ext[i].x) * margem;
        s_mesa_roi[i].y = s_mesa_ext[i].y + (cy - s_mesa_ext[i].y) * margem;
    }

    // Reconstrói a homografia baseada nos novos cantos
    prepara_homografia();
    prepara_luts();    // ATUALIZA a regiao de varredura para os novos cantos
    s_grid_ok = false; // Força recálculo da grade de iluminação
    
    ESP_LOGI(TAG, "MESA CALIBRADA AUTOMATICAMENTE (90 GRAUS):");
    ESP_LOGI(TAG, "  TL=(%.1f, %.1f) TR=(%.1f, %.1f)", s_mesa_ext[0].x, s_mesa_ext[0].y, s_mesa_ext[1].x, s_mesa_ext[1].y);
    ESP_LOGI(TAG, "  BR=(%.1f, %.1f) BL=(%.1f, %.1f)", s_mesa_ext[2].x, s_mesa_ext[2].y, s_mesa_ext[3].x, s_mesa_ext[3].y);
    return true;
}


// ---------- ajuste do sensor AO VIVO (testes de FPS) ----------
void Visao::ajustaExposicao(int delta) {
    sensor_t* s = esp_camera_sensor_get();
    if (!s) return;
    if (autoOn) {
        curAec = s->status.aec_value;
        curAgc = s->status.agc_gain;
    }
    curAec += delta;
    if (curAec < 0) curAec = 0;
    if (curAec > 1200) curAec = 1200;
    s->set_exposure_ctrl(s, 0);
    s->set_aec_value(s, curAec);
    autoOn = false;
    ESP_LOGI(TAG, "exposicao (aec) = %d  [manual]", curAec);
}

void Visao::ajustaGanho(int delta) {
    sensor_t* s = esp_camera_sensor_get();
    if (!s) return;
    if (autoOn) {
        curAec = s->status.aec_value;
        curAgc = s->status.agc_gain;
    }
    curAgc += delta;
    if (curAgc < 0) curAgc = 0;
    if (curAgc > 30) curAgc = 30;
    s->set_gain_ctrl(s, 0);
    s->set_agc_gain(s, curAgc);
    autoOn = false;
    ESP_LOGI(TAG, "ganho (agc) = %d  [manual] (clareia sem custar FPS, mas adiciona ruido)", curAgc);
}

void Visao::autoSensor(bool on) {
    sensor_t* s = esp_camera_sensor_get();
    if (!s) return;
    if (on) {
        aplica_sensor_auto();
    } else {
        curAec = s->status.aec_value > 0 ? s->status.aec_value : curAec;
        curAgc = s->status.agc_gain;
        aplica_sensor_fixo(curAec, curAgc);
    }
    autoOn = on;
    s_grid_ok = false;
    ESP_LOGI(TAG, "sensor AUTO = %s", on ? "ON (fps pode cair no escuro)" : "OFF (manual)");
}

void Visao::ajustaClock(int delta) {
    sensor_t* s = esp_camera_sensor_get();
    if (!s) return;
    if (curClkDiv < 0) curClkDiv = CAM_CLK_DIV_INICIAL;
    curClkDiv += delta;
    if (curClkDiv < 0)    curClkDiv = 0;
    if (curClkDiv > 0x3F) curClkDiv = 0x3F;
    // OV2640: banco DSP (0xFF=0x00), registrador R_DVP_SP (0xD3): divisor do PCLK.
    // menor divisor = clock mais rapido = mais FPS (risco de corromper frame).
    s->set_reg(s, 0xFF, 0xFF, 0x00);
    s->set_reg(s, 0xD3, 0xFF, curClkDiv);
    ESP_LOGW(TAG, "divisor de clock DVP (0xD3) = %d (menor=+rapido). Se a imagem corromper, aumente.", curClkDiv);
}

void Visao::configuraSensor(int aec, int agc, int clkdiv) {
    sensor_t* s = esp_camera_sensor_get();
    if (!s) return;
    if (aec >= 0) {
        s->set_exposure_ctrl(s, 0); s->set_aec_value(s, aec); curAec = aec; autoOn = false;
    }
    if (agc >= 0) {
        s->set_gain_ctrl(s, 0); s->set_agc_gain(s, agc); curAgc = agc;
    }
    if (clkdiv >= 0) {
        s->set_reg(s, 0xFF, 0xFF, 0x00);   // banco DSP
        s->set_reg(s, 0xD3, 0xFF, clkdiv); // divisor do PCLK (menor = +rapido)
        curClkDiv = clkdiv;
    }
}

void Visao::imprimeSensor() {
    sensor_t* s = esp_camera_sensor_get();
    if (!s) { ESP_LOGW(TAG, "sensor indisponivel"); return; }
    printf("=== SENSOR ===\n");
    printf("  auto=%d  aec(exposicao)=%d  agc(ganho)=%d  clkdiv(0xD3)=%d\n",
           autoOn ? 1 : 0, curAec, curAgc, curClkDiv);
    printf("  status: aec_value=%d agc_gain=%d\n", s->status.aec_value, s->status.agc_gain);
    printf("  modo captura: %s\n", s_modo_jpeg ? "JPEG" : (BOLA_MODO_COR ? "RGB565" : "GRAYSCALE"));
    printf("==============\n");
}

const VisaoDebugInfo& Visao::debugInfo() const { return dbg; }
