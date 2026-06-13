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
    ki  = PID_KI;
    kd  = PID_KD;
    tau = PID_TAU;
    dt  = PID_DT;
    integral = 0.0f;
    derivadaFiltrada.inicia(tau, dt);
}

float ControladorPID::correcao(float setpoint, float posicao) {
    float erro     = setpoint - posicao;
    float derivada = derivadaFiltrada.update(erro);

    // Termo integral com anti-windup: so acumula com Ki>0 e satura o acumulado
    // pra que ki*integral nunca passe de +-PID_I_MAX (evita estouro na borda).
    if (ki > 0.0f) {
        integral += erro * dt;
        float i_max_acc = PID_I_MAX / ki;
        if (integral >  i_max_acc) integral =  i_max_acc;
        if (integral < -i_max_acc) integral = -i_max_acc;
    } else {
        integral = 0.0f;
    }

    return kp * erro + ki * integral + kd * derivada;   // saida PID
}
