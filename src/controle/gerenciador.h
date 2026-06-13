#pragma once
#include "config.h"
#include "tipos.h"
#include "controle_pid.h"
#include "driver_atuacao.h"
#include "visao.h"
#include "fila_eventos.h"
// Modulo gerenciador: amarra visao + 2 PIDs + driver + fila de atuacao.

class ModuloGerenciador {
public:
    FilaDeEventos  fila;
    ControladorPID controladorX;
    ControladorPID controladorY;
    DriverAtuacao  driver;
    Visao          visao;

    float setpointX   = SETPOINT_X_PADRAO;
    float setpointY   = SETPOINT_Y_PADRAO;

    void inicia();
    // roda os PIDs e agenda atuacao. ativaX/ativaY permitem testar 1 eixo de
    // cada vez: o eixo desativado recebe 0 (mesa plana naquele eixo).
    void calculaAcaoControle(const Medicao& m, bool ativaX = true, bool ativaY = true);
    void processaFila();                          // executa as correcoes vencidas
};
