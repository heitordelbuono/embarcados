#pragma once
// Controlador PD com derivada filtrada (do seu controlePID.m / classes.c).

class FilteredDerivative {
public:
    float tau;             // Td/N
    float dt;
    float prev_input;
    float prev_derivative;

    void  inicia(float cte_tau, float deltat);
    float update(float current_input);   // retorna a derivada filtrada
};

class ControladorPID {
public:
    float kp;              // Kp
    float ki;              // Ki
    float kd;              // Kp*Td
    float tau;             // Td/N
    float dt;
    float integral;        // acumulador do termo integral (com anti-windup)
    FilteredDerivative derivadaFiltrada;

    void  inicia();
    void  resetIntegral() { integral = 0.0f; }
    float correcao(float setpoint, float posicao);   // sinal p/ o servo
};
