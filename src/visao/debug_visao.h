#pragma once
// Render de debug da visao: overlays sobre o frame e envio do frame anotado
// pela serial (PPM em base64). Tudo opera no frame de trabalho RGB565.
#include "esp_camera.h"
#include "visao.h"           // VisaoDebugInfo
#include "visao_interno.h"   // PontoF

void desenha_put_px(uint8_t* buf, int x, int y, uint16_t cor);
void desenha_retangulo(uint8_t* buf, int x0, int y0, int x1, int y1, uint16_t cor);
void desenha_cruz(uint8_t* buf, int x, int y, uint16_t cor);
void desenha_poligono(uint8_t* buf, const PontoF q[4], uint16_t cor);
void overlay_sobel(uint8_t* buf, const camera_fb_t* fb);
void envia_ppm_base64(const camera_fb_t* fb, const VisaoDebugInfo& d);
