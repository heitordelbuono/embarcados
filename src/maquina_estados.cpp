#include "maquina_estados.h"

Transicao matrizTransicao[NUM_ESTADOS][NUM_EVENTOS];

void iniciaMaquinaEstados() {
    // padrao: nenhuma acao, permanece no mesmo estado
    for (int i = 0; i < NUM_ESTADOS; i++)
        for (int j = 0; j < NUM_EVENTOS; j++)
            matrizTransicao[i][j] = { NENHUMA_ACAO, i };

    // ESPERANDO: botao liga o sistema
    matrizTransicao[ESPERANDO][EV_BOTAO]            = { A_LIGA,        SISTEMA_LIGADO };

    // SISTEMA_LIGADO
    matrizTransicao[SISTEMA_LIGADO][EV_DESEQUILIBRIO] = { A_EQUILIBRA,   EQUILIBRANDO };
    matrizTransicao[SISTEMA_LIGADO][EV_EQUILIBRIO]    = { A_EQUILIBRADO, EQUILIBRADO  };
    matrizTransicao[SISTEMA_LIGADO][EV_BOTAO]         = { A_DESLIGA,     ESPERANDO    };

    // EQUILIBRANDO
    matrizTransicao[EQUILIBRANDO][EV_EQUILIBRIO]   = { A_EQUILIBRADO, EQUILIBRADO    };
    matrizTransicao[EQUILIBRANDO][EV_BOTAO]        = { A_DESLIGA,     ESPERANDO      };
    matrizTransicao[EQUILIBRANDO][EV_BOLA_PERDIDA] = { NENHUMA_ACAO,  SISTEMA_LIGADO };

    // EQUILIBRADO
    matrizTransicao[EQUILIBRADO][EV_DESEQUILIBRIO] = { A_EQUILIBRA,  EQUILIBRANDO   };
    matrizTransicao[EQUILIBRADO][EV_BOTAO]         = { A_DESLIGA,    ESPERANDO      };
    matrizTransicao[EQUILIBRADO][EV_BOLA_PERDIDA]  = { NENHUMA_ACAO, SISTEMA_LIGADO };
}
