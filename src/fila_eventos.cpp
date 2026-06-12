#include "fila_eventos.h"

void FilaDeEventos::push(int64_t instante, int tipo, float dado) {
    if (numeroEventos == MAX_EVENTO) return;            // fila cheia
    for (int i = 0; i < numeroEventos; i++) {
        if (evento[i].instante > instante) {            // achei um evento posterior
            for (int j = numeroEventos; j > i; j--)     // empurra todos um lugar
                evento[j] = evento[j - 1];
            evento[i] = { tipo, dado, instante };       // insere no lugar certo
            numeroEventos++;
            return;
        }
    }
    evento[numeroEventos] = { tipo, dado, instante };   // insere no fim
    numeroEventos++;
}

DadosEvento FilaDeEventos::lerProximoEvento() {
    DadosEvento copia = evento[0];
    for (int i = 0; i < numeroEventos - 1; i++)
        evento[i] = evento[i + 1];
    numeroEventos--;
    return copia;
}
