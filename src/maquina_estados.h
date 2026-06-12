#pragma once
#include "tipos.h"
// Maquina de estados da mesa (sua maquinaEstados.c), via matriz de transicao.

struct Transicao {
    int acao;
    int proxEstado;
};

extern Transicao matrizTransicao[NUM_ESTADOS][NUM_EVENTOS];

void iniciaMaquinaEstados();
