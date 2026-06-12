#pragma once

class DriverAtuacao {
public:
    void iniciaMotores();
    void escreveAngulo(int canal, float angulo);
    void escrevePulso(int canal, int us);   // pulso direto em microsegundos
    void enviaCorrecaoX(float sinal);
    void enviaCorrecaoY(float sinal);
    void neutro();
    void debug();   // I2C scan + leitura de registradores do PCA9685
};
