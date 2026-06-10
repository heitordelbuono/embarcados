#include "classes/DriverDeAtuacao.h"

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