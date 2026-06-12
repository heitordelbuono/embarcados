#pragma once
#include <stdbool.h>
#include <stdint.h>

// Estados da mesa (baseado na sua maquinaEstados.c)
enum Estados { ESPERANDO, SISTEMA_LIGADO, EQUILIBRANDO, EQUILIBRADO, NUM_ESTADOS };

// Eventos que disparam transicoes
enum Eventos { EV_BOTAO, EV_DESEQUILIBRIO, EV_EQUILIBRIO, EV_BOLA_PERDIDA, NUM_EVENTOS };

// Acoes associadas as transicoes
enum Acoes { NENHUMA_ACAO = -1, A_LIGA, A_EQUILIBRA, A_EQUILIBRADO, A_DESLIGA };

// Tipos de evento agendado na fila (atuacao)
enum TiposFila { CORRECAO_X, CORRECAO_Y };

// Medicao da bola produzida pela visao
struct Medicao {
    float   x;       // ja com origem no centro
    float   y;
    bool    achou;
    int64_t t_us;    // timestamp (esp_timer_get_time)
};

// Evento agendado (sua filaDeEventos)
struct DadosEvento {
    int     tipo;
    float   dado;
    int64_t instante;   // micros em que deve executar
};
