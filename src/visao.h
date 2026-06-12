#pragma once
#include "tipos.h"
// Modulo de visao: configura a OV2640, captura frames e acha a bola.
// Produz uma Medicao (x, y com origem no centro, achou, timestamp).

class Visao {
public:
    bool    inicia();        // configura a camera (Etapa: camera)
    Medicao detecta();       // captura + threshold + centroide (Etapa: bola)
    void    calibraCor();    // auto-calibracao da cor (proximos passos)

private:
    float ultimoX = 0;       // ultima posicao (para o ROI - proximos passos)
    float ultimoY = 0;
    int   framesPerdidos = 0;
};
