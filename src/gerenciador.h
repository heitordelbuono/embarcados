#pragma once
#include "config.h"
#include "tipos.h"
#include "controle_pid.h"
#include "driver_atuacao.h"
#include "bluetooth.h"
#include "visao.h"
#include "fila_eventos.h"
// Modulo gerenciador: amarra visao + 2 PIDs + driver + bluetooth + fila + estados.

class ModuloGerenciador {
public:
    FilaDeEventos  fila;
    ControladorPID controladorX;
    ControladorPID controladorY;
    DriverAtuacao  driver;
    Bluetooth      bluetooth;
    Visao          visao;

    int   estadoAtual = ESPERANDO;
    float setpointX   = SETPOINT_X_PADRAO;
    float setpointY   = SETPOINT_Y_PADRAO;

    void inicia();
    void calculaAcaoControle(const Medicao& m);   // roda os PIDs e agenda atuacao
    void processaFila();                          // executa as correcoes vencidas
    void trataEvento(int evento);                 // avanca a maquina de estados
};
