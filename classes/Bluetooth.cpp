#include "classes/Bluetooth.h"

BluetoothSerial SerialBT;

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
