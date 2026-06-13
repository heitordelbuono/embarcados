#pragma once
#include "tipos.h"
// Fila de eventos ordenada por tempo (seu escalonador, da classes.c).
// Usada para agendar a atuacao dos servos (ex.: corrigir em +20 ms).

#define MAX_EVENTO 50

class FilaDeEventos {
public:
    DadosEvento evento[MAX_EVENTO];
    int numeroEventos = 0;

    void        push(int64_t instante, int tipo, float dado);
    bool        vazia() const { return numeroEventos == 0; }
    DadosEvento lerProximoEvento();    // remove e retorna o primeiro evento
};
