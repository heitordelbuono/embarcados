#pragma once

#include <stdio.h>

#define Kp 200 // cte proporcional do controlado PD
#define Td 1 // cte derivativa do controlador PD
#define N 100 // parâmetro do filtro do controlador PD
#define deltat 0.02 // discretização do tempo
#define L 0.095 // set point da bolinha

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