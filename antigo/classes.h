#pragma once

#include <stdio.h>
#include <ESP32Servo.h>
#include "BluetoothSerial.h"

#define MAX_EVENTO 50
#define pinoX 1 // pino do servo no eixo X
#define pinoY 2 // pino do servo no eixo Y
#define Kp 200 // cte proporcional do controlado PD
#define Td 1 // cte derivativa do controlador PD
#define N 100 // parâmetro do filtro do controlador PD
#define deltat 0.02 // discretização do tempo
#define L 0.095 // set point da bolinha

struct dados_eventos {
    int tipo;
    float dado;
    unsigned long instante;
};

enum TIPOS_FILA {CORRECAO_X, CORRECAO_Y};

// MÓDULO FILA DE EVENTOS

class FilaDeEventos{
    public:
    struct dados_eventos Evento[MAX_EVENTO];
    int numeroEventos = 0;

    void push(unsigned long instante, int tipo, float dado);
    dados_eventos lerProximoEvento(void);
};

// MÓDULO CONTROLE BLUETOOTH

class Bluetooth {
    public:
    float pos_x_atual;
    float pos_y_atual;
    FilaDeEventos fila;
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

// módulo controlador PD

class FilteredDerivative{
    public:
    float tau; //Td/N
    float dt;
    float prev_input; // input anterior
    float prev_derivative; // derivada anterior

    void iniciaFiltro(void);

    float update(float current_input);
};

class ControladorPID {
    public:
    float cteProporcional; //Kp
    float cteDerivativa; // Kp*Td
    float tau; // Td/N
    float dt;
    FilteredDerivative filtered_derivative;

    // método que inicia o controlador com os parâmetros desejados
    void iniciaControlador(void);

    float correcao(float setpoint, float posicao);
};

// módulo driver de atuação

class DriverDeAtuacao{
    public:
    Servo servoX;
    Servo servoY;

    void iniciaMotores(void); // inicia motores

    void enviaCorrecaoX(float sinal_correcao); // envia correcao pro X

    void enviaCorrecaoY(float sinal_correcao); // envia correcao pro Y

};

// módulo gerenciador

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