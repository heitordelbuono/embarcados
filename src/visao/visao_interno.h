#pragma once
// Tipos e leitura de pixel compartilhados entre visao.cpp e debug_visao.cpp.
// O frame de trabalho e SEMPRE RGB565 (JPEG decodificado on-chip).
#include "esp_camera.h"

struct PontoF { float x; float y; };

static inline uint16_t rgb565_at(const uint8_t* buf, int idx) {
    return ((uint16_t)buf[2 * idx] << 8) | buf[2 * idx + 1];
}
static inline void rgb565_split(uint16_t p, int& r, int& g, int& b) {
    r = ((p >> 11) & 0x1F) * 255 / 31;
    g = ((p >> 5)  & 0x3F) * 255 / 63;
    b = ( p        & 0x1F) * 255 / 31;
}
static inline int luma_from_rgb(int r, int g, int b) { return (77 * r + 150 * g + 29 * b) >> 8; }

static inline int pixel_luma(const camera_fb_t* fb, int idx) {
    int r, g, b; rgb565_split(rgb565_at(fb->buf, idx), r, g, b);
    return luma_from_rgb(r, g, b);
}
