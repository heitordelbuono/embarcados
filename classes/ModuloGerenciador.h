#pragma once 

#include <stdio.h>
#include "classes/Bluetooth.h"
#include "classes/ControladorPID.h"
#include "classes/DriverDeAtuacao.h"
#include "classes/FilaDeEventos.h"

class ModuloGerenciador{
    public:
    FilaDeEventos fila; // fila de eventos
    ControladorPID controladorX; // controlador da direção x
    ControladorPID controladorY; // controlador da direção y
    DriverDeAtuacao driver; // driver de atuação dos servos
    Bluetooth bluetooth; // bluetooth (botão + por onde recebe a posição da bolinha)

    void iniciaModuloGerenciador(void);

    void calculaAcaoControle_emX(float L, float posicaoX);

    void calculaAcaoControle_emY(float L, float posicaoY);

    void calibra(void);
};