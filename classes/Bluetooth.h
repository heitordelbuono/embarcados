#pragma once

#include <stdio.h>
#include "BluetoothSerial.h"

class Bluetooth {
    public:
    float pos_x_atual;
    float pos_y_atual;
    bool sistema_ligado = false;
    bool flag_calibra = false;
    bool botao_lido = false;

    // incializa o modulo bluetooth
    void iniciaConexao(void);

    // verifica se tem dado novo
    void temDadoNovo(void);

    // pega a posição x atual
    float getPosicaoX(void);

    // pega posição y atual
    float getPosicaoY(void);

    // método para o botão liga/desliga
    void botao_LigaDesliga(void);

    // método para o botão de calibragem
    void botao_Calibra(void);

};
