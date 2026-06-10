#pragma once 

#include <stdio.h>
#include <ESP32Servo.h>

#define pinoX 1 // pino do servo no eixo X
#define pinoY 2 // pino do servo no eixo Y

class DriverDeAtuacao{
    public:
    Servo servoX;
    Servo servoY;

    void iniciaMotores(void); // inicia motores

    void enviaCorrecaoX(float sinal_correcao); // envia correcao pro X

    void enviaCorrecaoY(float sinal_correcao); // envia correcao pro Y

};