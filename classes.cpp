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

struct dados_eventos {
    int tipo;
    float dado;
    unsigned long instante;
};

enum TIPOS_FILA {CORRECAO_X, CORRECAO_Y};

using namespace std;

// MÓDULO DA FILA DE EVENTOS

class FilaDeEventos {
    public:
    struct dados_eventos Evento[MAX_EVENTO];
    int numeroEventos = 0;

    void push(unsigned long instante, int tipo, float dado){
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

    dados_eventos lerProximoEvento(void){ // retorna o struct do primeiro evento da fila de eventos
        dados_eventos copia_evento = Evento[0]; // faz uma copia do primeiro evento da fila
        for (int i=0; i < numeroEventos-1; i++){ // move todos os eventos uma posição
            Evento[i] = Evento[i+1];
        }
        numeroEventos--; // decrementa o numero de eventos
        return copia_evento;
    }

};

// MÓDULO DO CONTROLE BLUETOOTH

BluetoothSerial SerialBT;

class Bluetooth {
    public:
    float pos_x_atual;
    float pos_y_atual;
    FilaDeEventos fila;
    bool sistema_ligado = false;
    bool flag_calibra = false;

    // inicializa módulo bluetooth
    void iniciaConexao(void){
        SerialBT.begin("Mesa PID");
    }

    // verifica se tem dado novo
    bool temDadoNovo(){
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
            return true;
        } else {
            return false;
        }
    }

    // pega a posição x atual
    float getPosicaoX(void){
        return pos_x_atual;
    }

    // pega a posição y atual
    float getPosicaoY(void){
        return pos_y_atual;
    }

    // MÉTODO PARA O BOTÃO LIGA/DESLIGA
    void botao_LigaDesliga(void){
        sistema_ligado = !sistema_ligado;
    }

    // MÉTODO PARA O BOTÃO DE CALIBRAGEM
    void botao_Calibra(void){
        flag_calibra = true;
    }
};

// MÓDULO DO CONTROLADOR PID
class FilteredDerivative{ // classe responsável por fazer a filtragem da derivada (passa baixas)
    public:
    float tau; // Td/N
    float dt;
    float prev_input; // input anterior
    float prev_derivative; // derivada anterior

    void iniciaFiltro (){
        tau = Td/N;
        dt = deltat;
        prev_input = 0;
        prev_derivative = 0;
    }

    float update(float current_input){ // método que retorna a derivada filtrada
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
};

class ControladorPID {
    public:
    float cteProporcional; // Kp
    float cteDerivativa; // Kp*Td
    float tau; // Td/N
    float dt;
    FilteredDerivative filtered_derivative;

    void iniciaControlador(){ // método que inicia o controldor com os parâmetros desejados
        filtered_derivative.iniciaFiltro();
        cteProporcional = Kp;
        cteDerivativa = Kp*Td;
        tau = Td/N; // Td/N
        dt = deltat; // discretização do tempo
    }

    float correcao(float setpoint, float posicao){ // obtém sinal de correção para os servo motores
        float error = setpoint - posicao; // erro entre set-point e posição atual
        // aplica derivada filtrada
        float derivada = filtered_derivative.update(error);

        // saída de controle PD
        float output = cteProporcional*error + cteDerivativa*derivada;

        return output;
    }
};

// MÓDULO DO DRIVER DE ATUAÇÃO
class DriverDeAtuacao {
    public:
    Servo servoX;
    Servo servoY;

    void iniciaMotores(void){
        servoX.attach(pinoX);
        servoY.attach(pinoY);
    }

    void enviaCorrecaoX(float sinal_correcao){
        float anguloX = 90 + sinal_correcao; // corrige em relação ao centro do motor
        if (anguloX > 110){ // saturação
            anguloX = 110;
        }
        if (anguloX < 70){
            anguloX = 70;
        }
        servoX.write(anguloX);
    }

    void enviaCorrecaoY(float sinal_correcao){
        float anguloY = 90 + sinal_correcao; // corrige em relação ao centro do motor
        if (anguloY > 110){ // saturacao
            anguloY = 110;
        }
        if (anguloY < 70){
            anguloY = 70;
        }
        servoY.write(anguloY);
    }
};

// MODULO GERENCIADOR
class ModuloGerenciador {
    public:
    FilaDeEventos fila; // fila de eventos
    ControladorPID controladorX; // controlador da direção x
    ControladorPID controladorY; // controlador da direção y
    DriverDeAtuacao driver; // driver de atuação dos servos
    Bluetooth bluetooth; // bluetooth (botão + por onde recebe a posição da bolinha)

    void iniciaModuloGerenciador(void){
        controladorX.iniciaControlador();
        controladorY.iniciaControlador();
        bluetooth.iniciaConexao();
        driver.iniciaMotores();
    }

    void calculaAcaoControle_emX(float L, float posicaoX){
        float sinal_x = controladorX.correcao(L, posicaoX);
        unsigned long tempo_atual = micros();
        fila.push(tempo_atual + 20000, CORRECAO_X, sinal_x);
    }

    void calculaAcaoControle_emY(float L, float posicaoY){
        float sinal_y = controladorY.correcao(L, posicaoY);
        unsigned long tempo_atual = micros();
        fila.push(tempo_atual + 20000, CORRECAO_Y, sinal_y);

    }

    void calibra(void){
        driver.servoX.write(90);
        driver.servoY.write(90);
    }
};
