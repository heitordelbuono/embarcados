#include "classes/ControladorPID.h"

// MÓDULO DO CONTROLADOR PID

    void FilteredDerivative::iniciaFiltro (void){
        tau = Td/N;
        dt = deltat;
        prev_input = 0;
        prev_derivative = 0;
    }

    float FilteredDerivative::update(float current_input){ // método que retorna a derivada filtrada
        // diferença finita
        float raw_derivative = (current_input - prev_input)/dt;

        // coeficiente do filtro
        float alpha = tau/(tau + dt);

        // aplicar filtro passa baixa
        float filtered_derivative = alpha*prev_derivative + (1 - alpha)*raw_derivative;

        // salva estado
        prev_input = current_input;
        prev_derivative = filtered_derivative;

        return filtered_derivative;
    }



    void ControladorPID::iniciaControlador(void){ // método que inicia o controldor com os parâmetros desejados
        filtered_derivative.iniciaFiltro();
        cteProporcional = Kp;
        cteDerivativa = Kp*Td;
        tau = Td/N; // Td/N
        dt = deltat; // discretização do tempo
    }

    float ControladorPID::correcao(float setpoint, float posicao){ // obtém sinal de correção para os servo motores
        float error = setpoint - posicao; // erro entre set-point e posição atual
        // aplica derivada filtrada
        float derivada = filtered_derivative.update(error);

        // saída de controle PD
        float output = cteProporcional*error + cteDerivativa*derivada;

        return output;
    }
