#include "maquinaEstados.h"

struct proximo matrizTransicaoEstados[NUM_ESTADOS][NUM_EVENTOS];

void iniciaMaquinaEstados(void){
    int i, j;
    for (i=0; i<NUM_ESTADOS; i++){
        for (j=0; j<NUM_EVENTOS; j++){
            matrizTransicaoEstados[i][j].acao = NENHUMA_ACAO;
            matrizTransicaoEstados[i][j].prox_estado = i;
        }
    }
    // ESTADO ESPERANDO
    matrizTransicaoEstados[ESPERANDO][BOTAO].acao = A01;
    matrizTransicaoEstados[ESPERANDO][BOTAO].prox_estado = SISTEMA_LIGADO;

    // ESTADO SISTEMA LIGADO

        // DETECTA DESEQUILIBRIO
    matrizTransicaoEstados[SISTEMA_LIGADO][DESEQUILIBRIO].acao = A02;
    matrizTransicaoEstados[SISTEMA_LIGADO][DESEQUILIBRIO].prox_estado = EQUILIBRANDO;

        // DETECTA EQUILIBRIO
    matrizTransicaoEstados[SISTEMA_LIGADO][EQUILIBRIO].acao = A03;
    matrizTransicaoEstados[SISTEMA_LIGADO][EQUILIBRIO].prox_estado = EQUILIBRADO;

    // EQUILIBRANDO

        // APERTA BOTAO
    matrizTransicaoEstados[EQUILIBRANDO][BOTAO].acao = A04;
    matrizTransicaoEstados[EQUILIBRANDO][BOTAO].prox_estado = ESPERANDO;

        // DETECTA EQUILIBRIO
    matrizTransicaoEstados[EQUILIBRANDO][EQUILIBRIO].acao = A03;
    matrizTransicaoEstados[EQUILIBRANDO][EQUILIBRIO].prox_estado = EQUILIBRADO;

    // EQUILIBRADO
        
        // APERTA BOTAO
    matrizTransicaoEstados[EQUILIBRADO][BOTAO].acao = A04;
    matrizTransicaoEstados[EQUILIBRADO][BOTAO].prox_estado = ESPERANDO;

        // DETECTA DESEQUILIBRIO
    matrizTransicaoEstados[EQUILIBRADO][DESEQUILIBRIO].acao = A02;
    matrizTransicaoEstados[EQUILIBRADO][DESEQUILIBRIO].prox_estado = EQUILIBRANDO;
}
