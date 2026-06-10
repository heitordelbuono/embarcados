#include "classes.h"

BluetoothSerial SerialBT;

// MÓDULO DA FILA DE EVENTOS

void FilaDeEventos::push(unsigned long instante, int tipo, float dado){
        if (numeroEventos == MAX_EVENTO){ // se a fila estiver cheia retorno
        return;
        } else if (numeroEventos == 0){ // se a fila estiver vazia insiro na primeira posição
            Evento[numeroEventos].instante = instante;
            Evento[numeroEventos].tipo = tipo;
            Evento[numeroEventos].dado = dado;
            numeroEventos++;
            return;
        } else {
            for (int i=0; i < numeroEventos; i++){ // percorro a fila a partir da primeira posição
                if (Evento[i].instante > instante){ // se houver um evento que aconteceu depois do evento que quero inserir
                    for(int j = numeroEventos; j > i; j--){ // movo este evento e todos depois dele para a posição seguinte
                            Evento[j] = Evento[j-1];
                    } // e insiro o meu dado na posição onde ele estava
                    Evento[i].instante = instante;
                    Evento[i].tipo = tipo;
                    Evento[i].dado = dado;
                    numeroEventos++;
                    return;
                }
            } // se eu percorrer toda a fila e nao encontrar nenhum evento que aconteceu depois do meu
            // insiro meu dado na última posição
            Evento[numeroEventos].instante = instante;
            Evento[numeroEventos].tipo = tipo;
            Evento[numeroEventos].dado = dado;
            numeroEventos++;
            return;
        }
    }

    dados_eventos FilaDeEventos::lerProximoEvento(void){ // retorna o struct do primeiro evento da fila de eventos
        dados_eventos copia_evento = Evento[0]; // faz uma copia do primeiro evento da fila
        for (int i=0; i < numeroEventos-1; i++){ // move todos os eventos uma posição
            Evento[i] = Evento[i+1];
        }
        numeroEventos--; // decrementa o numero de eventos
        return copia_evento;
    }



// MÓDULO DO CONTROLE BLUETOOTH

    void Bluetooth::iniciaConexao(void){
        SerialBT.begin("Mesa PID");
    }

    // verifica se tem dado novo
    void Bluetooth::temDadoNovo(void){
        if (SerialBT.available()){
            String mensagem = SerialBT.readStringUntil('\n');
            mensagem.trim();
            if (mensagem == "CALIBRA"){
                botao_Calibra();
            } else if (mensagem == "LIGA"){
                botao_LigaDesliga();
            } else {
                sscanf(mensagem.c_str(), "#%f$%f", &pos_x_atual, &pos_y_atual);
            }
        }
        return;
    }

    // pega a posição x atual
    float Bluetooth::getPosicaoX(void){
        return pos_x_atual;
    }

    // pega a posição y atual
    float Bluetooth::getPosicaoY(void){
        return pos_y_atual;
    }

    // MÉTODO PARA O BOTÃO LIGA/DESLIGA
    void Bluetooth::botao_LigaDesliga(void){
        sistema_ligado = !sistema_ligado;
        botao_lido = false;
    }

    // MÉTODO PARA O BOTÃO DE CALIBRAGEM
    void Bluetooth::botao_Calibra(void){
        flag_calibra = true;
    }


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

// MÓDULO DO DRIVER DE ATUAÇÃO

    void DriverDeAtuacao::iniciaMotores(void){
        servoX.attach(pinoX);
        servoY.attach(pinoY);
    }

    void DriverDeAtuacao::enviaCorrecaoX(float sinal_correcao){
        float anguloX = 90 + sinal_correcao; // corrige em relação ao centro do motor
        if (anguloX > 110){ // saturação
            anguloX = 110;
        }
        if (anguloX < 70){
            anguloX = 70;
        }
        servoX.write(anguloX);
    }

    void DriverDeAtuacao::enviaCorrecaoY(float sinal_correcao){
        float anguloY = 90 + sinal_correcao; // corrige em relação ao centro do motor
        if (anguloY > 110){ // saturacao
            anguloY = 110;
        }
        if (anguloY < 70){
            anguloY = 70;
        }
        servoY.write(anguloY);
    }


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

