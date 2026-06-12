#pragma once

class DriverAtuacao {
public:
    void iniciaMotores();
    void desconecta();   // solta o PCA do barramento (p/ a camera poder reinit)
    bool trocaEixos();   // inverte canais X<->Y em runtime; retorna true se trocado
    void escreveAngulo(int canal, float angulo);
    void escrevePulso(int canal, int us);   // pulso direto em microsegundos
    void enviaCorrecaoX(float sinal);
    void enviaCorrecaoY(float sinal);
    void neutro();
    void ajustaNeutroX(float delta);
    void ajustaNeutroY(float delta);
    void resetNeutro();
    void salvaNeutro();
    float neutroX() const;
    float neutroY() const;
    void debug();   // I2C scan + leitura de registradores do PCA9685
};
