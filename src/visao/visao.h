#pragma once
#include "tipos.h"
// Modulo de visao: configura a OV2640, captura frames e acha a bola.
// Produz uma Medicao (x, y em cm com origem no centro da mesa, achou, timestamp).
//
// Pipeline (detalhes em docs/VISAO.md):
//   captura grayscale -> referencia local (grade ou fundo) -> candidatos
//   -> componentes conexos (run-length + union-find) -> escolhe o blob
//   -> centroide ponderado (sub-pixel) -> homografia px->cm -> filtro alfa-beta.

struct VisaoDebugInfo {
    bool  achou = false;
    int   cx = 0;            // centroide em pixels (sub-pixel arredondado)
    int   cy = 0;
    float cx_f = 0.0f;       // centroide em pixels (float, sub-pixel)
    float cy_f = 0.0f;
    int   area = 0;          // area estimada do blob (pixels)
    int   x0 = 0;            // bbox do blob
    int   y0 = 0;
    int   x1 = 0;
    int   y1 = 0;
    int   roi_x0 = 0;        // janela varrida (tracking ou ROI inteira)
    int   roi_y0 = 0;
    int   roi_x1 = 0;
    int   roi_y1 = 0;
    int   media_y = 0;       // referencia media da mesa (informativo)
    int   blobs = 0;         // quantos componentes conexos foram avaliados
    int   runs = 0;          // quantos segmentos horizontais (carga do frame)
    bool  overflow = false;  // estourou VISAO_MAX_RUNS (limiar frouxo demais)
    bool  usou_fundo = false;// referencia veio do fundo capturado
    float mesa_x_cm = 0.0f;  // posicao crua (antes do filtro)
    float mesa_y_cm = 0.0f;
    float filt_x_cm = 0.0f;  // posicao filtrada (alfa-beta)
    float filt_y_cm = 0.0f;
    float vel_x_cm = 0.0f;
    float vel_y_cm = 0.0f;
    int64_t captura_us = 0;
    int64_t processo_us = 0;
    int64_t dt_us = 0;
};

class Visao {
public:
    bool    inicia();        // configura a camera + geometria (homografia)
    Medicao detecta();       // captura + segmentacao + centroide + cm + filtro
    void    solicitaDebug(); // manda o proximo frame anotado pela serial
    void    capturaFundo();  // memoriza a mesa VAZIA como referencia (comando 'g')
    void    limpaFundo();    // volta para a referencia por grade local
    void    calibraCamera(); // congela exposicao/ganho/AWB (e salva na NVS)
    void    resetFiltro();   // zera o filtro alfa-beta
    void    debugSobel(bool on); // overlay de bordas (Sobel) no frame de debug
    void    debugMascara(bool on); // frame debug preto/branco com a mascara de candidatos
    bool    calibraMesaGeometrica(); // calibra mesa automaticamente detectando as bordas
    void    ajustaCentroPx(float dx, float dy); // desloca origem em pixels (ajuste manual)
    void    ajustaCentroCm(float dx, float dy); // desloca origem em cm (ajuste manual fino)
    void    salvaCentroPx();                    // salva origem manual na NVS
    void    resetCentroPx();                    // volta origem ao centro da homografia
    void    imprimeCentroPx();                  // mostra offset manual atual

    // --- ajuste do sensor AO VIVO (testes de FPS, pontos 5 e 6) ---
    void    ajustaExposicao(int delta);  // muda o tempo de exposicao (aec_value)
    void    ajustaGanho(int delta);      // muda o ganho (agc_gain), clareia sem custar FPS
    void    autoSensor(bool on);         // liga/desliga AEC/AGC/AWB automaticos
    void    ajustaClock(int delta);      // muda o divisor de clock DVP (ponto 6)
    void    configuraSensor(int aec, int agc, int clkdiv); // set absoluto (-1 = nao mexe)
    void    imprimeSensor();             // mostra aec/agc/auto/divisor atuais

    const VisaoDebugInfo& debugInfo() const;

private:
    // tracking / ultima posicao
    float ultimoX = 0;
    float ultimoY = 0;
    int   ultimoCx = 0;
    int   ultimoCy = 0;
    int   framesPerdidos = 0;

    // referencia adaptativa (grade) — recalculo periodico
    int   framesAteMedia = 0;
    int   mediaMesaY = 0;

    // filtro alfa-beta (estado em cm)
    bool  filtroIniciado = false;
    float fx = 0, fy = 0;     // posicao filtrada
    float fvx = 0, fvy = 0;   // velocidade filtrada
    int64_t ultimoT = 0;      // timestamp da medicao anterior

    void  aplicaEstadoSensor();   // (re)aplica exposicao/ganho apos (re)init da camera

    bool  debugSolicitado = false;
    bool  sobelLigado = false;
    bool  mascaraLigada = false;

    // estado do sensor (ajuste ao vivo)
    int   curAec = 0;
    int   curAgc = 0;
    int   curClkDiv = -1;
    bool  autoOn = false;

    VisaoDebugInfo dbg;
};
