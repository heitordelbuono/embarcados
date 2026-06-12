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

// Medicao da bola produzida pela visao.
// x,y ja vem em cm com origem no centro da mesa (via homografia).
// Quando FILTRO_ATIVO, x,y sao filtrados (alfa-beta) e vx,vy sao a
// velocidade estimada em cm/s (uteis depois para o termo D do PID).
struct Medicao {
    float   x;       // cm, origem no centro da mesa
    float   y;
    float   vx;      // cm/s (0 se filtro desligado)
    float   vy;
    bool    achou;
    float   dt;      // s, intervalo desde a medicao anterior
    int64_t t_us;    // timestamp (esp_timer_get_time)
};

// Evento agendado (sua filaDeEventos)
struct DadosEvento {
    int     tipo;
    float   dado;
    int64_t instante;   // micros em que deve executar
};
