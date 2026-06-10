#include "classes/FilaDeEventos.h"

// MÓDULO DA FILA DE EVENTOS

void FilaDeEventos::push(unsigned long instante, int tipo, float dado){
        if (numeroEventos == MAX_EVENTO){ // se a fila estiver cheia retorno
        return;
        } else if (numeroEventos == 0){ // se a fila estiver vazia insiro na primeira posição
            Evento[numeroEventos].instante = instante;
            Evento[numeroEventos].tipo = tipo;
            Evento[numeroEventos].dado = dado;
            numeroEventos++;
            return;
        } else {
            for (int i=0; i < numeroEventos; i++){ // percorro a fila a partir da primeira posição
                if (Evento[i].instante > instante){ // se houver um evento que aconteceu depois do evento que quero inserir
                    for(int j = numeroEventos; j > i; j--){ // movo este evento e todos depois dele para a posição seguinte
                            Evento[j] = Evento[j-1];
                    } // e insiro o meu dado na posição onde ele estava
                    Evento[i].instante = instante;
                    Evento[i].tipo = tipo;
                    Evento[i].dado = dado;
                    numeroEventos++;
                    return;
                }
            } // se eu percorrer toda a fila e nao encontrar nenhum evento que aconteceu depois do meu
            // insiro meu dado na última posição
            Evento[numeroEventos].instante = instante;
            Evento[numeroEventos].tipo = tipo;
            Evento[numeroEventos].dado = dado;
            numeroEventos++;
            return;
        }
    }

    dados_eventos FilaDeEventos::lerProximoEvento(void){ // retorna o struct do primeiro evento da fila de eventos
        dados_eventos copia_evento = Evento[0]; // faz uma copia do primeiro evento da fila
        for (int i=0; i < numeroEventos-1; i++){ // move todos os eventos uma posição
            Evento[i] = Evento[i+1];
        }
        numeroEventos--; // decrementa o numero de eventos
        return copia_evento;
    }
