#pragma once

#define NUM_EVENTOS 3
#define NUM_ESTADOS 4

enum ESTADOS {ESPERANDO, SISTEMA_LIGADO, EQUILIBRANDO, EQUILIBRADO};
enum ACOES { A01, A02, A03, A04, NENHUMA_ACAO};
enum EVENTOS {BOTAO, DESEQUILIBRIO, EQUILIBRIO, NENHUM_EVENTO};
extern struct proximo {
    int acao;
    int prox_estado;
};
struct proximo matrizTransicaoEstados[NUM_ESTADOS][NUM_EVENTOS];

void iniciaMaquinaEstados(void); // preenche a máquina de estados com as ações e próximos estados par cada evento