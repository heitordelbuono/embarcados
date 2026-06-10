#include "classes/ModuloGerenciador.h"

// MODULO GERENCIADOR

    void ModuloGerenciador::iniciaModuloGerenciador(void){
        controladorX.iniciaControlador();
        controladorY.iniciaControlador();
        bluetooth.iniciaConexao();
        driver.iniciaMotores();
    }

    void ModuloGerenciador::calculaAcaoControle_emX(float L, float posicaoX){
        float sinal_x = controladorX.correcao(L, posicaoX);
        unsigned long tempo_atual = micros();
        driver.servoX.write(sinal_x);
        // fila.push(tempo_atual + 20000, CORRECAO_X, sinal_x);
    }

    void ModuloGerenciador::calculaAcaoControle_emY(float L, float posicaoY){
        float sinal_y = controladorY.correcao(L, posicaoY);
        unsigned long tempo_atual = micros();
        driver.servoY.write(sinal_y);
        // fila.push(tempo_atual + 20000, CORRECAO_Y, sinal_y);

    }

    void ModuloGerenciador::calibra(void){
        driver.servoX.write(90);
        driver.servoY.write(90);
    }

