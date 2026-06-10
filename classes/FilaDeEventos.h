#pragma once

#include <stdio.h>
#define MAX_EVENTO 50

struct dados_eventos {
    int tipo;
    float dado;
    unsigned long instante;
};

enum TIPOS_FILA {CORRECAO_X, CORRECAO_Y};

// MÓDULO FILA DE EVENTOS

class FilaDeEventos{
    public:
    struct dados_eventos Evento[MAX_EVENTO];
    int numeroEventos = 0;

    void push(unsigned long instante, int tipo, float dado);
    dados_eventos lerProximoEvento(void);
};