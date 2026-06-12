#include "controle_pid.h"
#include "config.h"

void FilteredDerivative::inicia(float cte_tau, float deltat) {
    tau = cte_tau;
    dt  = deltat;
    prev_input = 0;
    prev_derivative = 0;
}

float FilteredDerivative::update(float current_input) {
    float raw   = (current_input - prev_input) / dt;     // diferenca finita
    float alpha = tau / (tau + dt);                      // coef. do filtro
    float filt  = alpha * prev_derivative + (1.0f - alpha) * raw;
    prev_input      = current_input;
    prev_derivative = filt;
    return filt;
}

void ControladorPID::inicia() {
    kp  = PID_KP;
    kd  = PID_KD;
    tau = PID_TAU;
    dt  = PID_DT;
    derivadaFiltrada.inicia(tau, dt);
}

float ControladorPID::correcao(float setpoint, float posicao) {
    float erro     = setpoint - posicao;
    float derivada = derivadaFiltrada.update(erro);
    return kp * erro + kd * derivada;     // saida PD
}
