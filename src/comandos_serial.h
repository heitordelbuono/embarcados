#pragma once
#include "config.h"
#include "gerenciador.h"

// Estado da interface pela serial (comandos do teclado). Compartilhado entre
// o loop principal (que atua nos servos a cada frame) e o parser de comandos.
struct EstadoUI {
    bool  pidX = false, pidY = false;        // PID por eixo (o/ox/oy)
    bool  manualAtivo = false;               // segura um angulo fixo (x/y)
    float manualX = 0.0f, manualY = 0.0f;
    bool  dancaX = false, dancaY = false;    // gangorra senoidal por eixo (w/b)
    float danceFaseX = 0.0f, danceFaseY = 0.0f, danceAmp = 0.0f;

    bool  sobel = false;
    bool  mascara = false;
    bool  telemetria = false;
    bool  autoSens = (CAM_AUTO_AJUSTE != 0);
    float fps_atual = 0.0f;

    // parser de comandos com argumento (x<val>, oy, zx<n>, ...)
    char  linebuf[16];
    int   linepos = 0;
    bool  capturando = false;
};

void comandos_ajuda(ModuloGerenciador& g);

// Le todos os caracteres disponiveis na serial e aplica os comandos,
// mutando o estado da UI e o gerenciador. Nao bloqueia.
void comandos_processa(ModuloGerenciador& g, EstadoUI& ui);
