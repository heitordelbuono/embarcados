// ============================================================
//  comandos_serial.cpp - interface pela serial (teclado)
//  Ajuda + parser dos comandos. A atuacao nos servos a cada frame
//  (PID/danca/manual/neutro) fica no loop principal de main.cpp.
// ============================================================
#include "comandos_serial.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <stdlib.h>

void comandos_ajuda(ModuloGerenciador& g) {
    printf("\n=== MESA PID (modo normal de execucao) ===\n");
    printf(" Deteccao/coordenada:\n");
    printf("  f -> frame anotado pela serial (imagem original + overlays)\n");
    printf("  g -> captura o FUNDO (mesa VAZIA) -> ativa subtracao de fundo\n");
    printf("  G -> descarta o fundo (volta para grade de iluminacao)\n");
    printf("  e -> liga/desliga overlay Sobel no frame de debug\n");
    printf("  h -> liga/desliga debug de mascara (branco=candidato, preto=resto)\n");
    printf("  r -> reinicia o filtro alfa-beta\n");
    printf("  t -> liga/desliga telemetria CSV (t,x,y,vx,vy,achou,fps)\n");
    printf(" Captura / sensor / FPS:\n");
    printf("  + / - -> exposicao (aec) +/- %d  (menor = +FPS, mais escuro)\n", CAM_AEC_STEP);
    printf("  . / , -> ganho (agc) +/- 1       (clareia sem custar FPS, +ruido)\n");
    printf("  a     -> liga/desliga AUTO (exposicao/ganho/AWB)\n");
    printf("  k / l -> divisor de clock DVP - / +  (manual; pode travar -> reboot)\n");
    printf("  c     -> calibra a camera (auto -> congela -> salva NVS)\n");
    printf("  m     -> calibra geometria da mesa (detecta cantos e homografia)\n");
    printf("  0cx<n>/0cy<n> -> desloca centro em passos de 0.1 cm (ex: 0cx1 0cy-2)\n");
    printf("  ux<n> / uy<n> -> desloca centro/origem em pixels (ex: ux2 uy-1)\n");
    printf("  us / u0       -> salva / reseta centro manual (0s salva tudo, 00 reseta tudo)\n");
    printf("  q     -> imprime estado do sensor (aec/agc/auto/clkdiv) e ganhos PID\n");
    printf(" Servo na mao (testa hardware, coordenada continua rodando):\n");
    printf("  x<val> -> servo X manual, -%d..+%d graus + Enter (ex: x-10  x15)\n", SERVO_RANGE, SERVO_RANGE);
    printf("  y<val> -> servo Y manual, -%d..+%d graus + Enter (ex: y-10  y15)\n", SERVO_RANGE, SERVO_RANGE);
    printf("  w     -> liga/desliga DANCA no eixo X (gangorra senoidal)\n");
    printf("  b     -> liga/desliga DANCA no eixo Y  (w+b = X+Y juntos)\n");
    printf("  n     -> NEUTRO: para manual/danca/PID e zera a mesa\n");
    printf("  T     -> TROCA os canais dos servos X<->Y (na hora)\n");
    printf("  0x<n> / 0y<n> -> ajusta zero do servo em graus (ex: 0x1 0y-1)\n");
    printf("  zx<n> / zy<n> -> ajusta neutro do servo em graus (ex: zx1 zy-0.5)\n");
    printf("  zs / z0       -> salva / reseta neutro dos servos\n");
    printf(" Controle / PID (precisa Enter):\n");
    printf("  o  -> liga/desliga PID nos DOIS eixos\n");
    printf("  ox -> PID so no eixo X (Y fica plano)  |  oy -> so no eixo Y\n");
    printf("  P / p -> Aumentar/Diminuir Proporcional (Kp) em 1\n");
    printf("  I / i -> Aumentar/Diminuir Integral (Ki) em 0.1\n");
    printf("  D / d -> Aumentar/Diminuir Derivativo (Kd) em 0.1\n");
    printf("  s     -> Varredura rapida dos servos (-10 -> 0 -> +10 -> 0)\n");
    printf("  v     -> Depuracao dos servos (I2C scan + registradores)\n");
    printf("  ?     -> esta ajuda\n");
    printf("Modo atual: JPEG | print a cada 5s\n\n");
    (void)g;
}

void comandos_processa(ModuloGerenciador& g, EstadoUI& ui) {
    int c;
    while ((c = getchar()) >= 0) {
        // --- comandos com argumento terminados por Enter ---
        if (ui.capturando) {
            if (c == '\n' || c == '\r') {
                ui.linebuf[ui.linepos] = '\0';
                char cmd = ui.linebuf[0];
                if (cmd == 'o' || cmd == 'O') {
                    // o = alterna os 2 eixos | ox = so X | oy = so Y
                    char ax = (ui.linepos > 1) ? ui.linebuf[1] : '\0';
                    ui.dancaX = ui.dancaY = false; ui.manualAtivo = false;
                    if      (ax == 'x' || ax == 'X') { ui.pidX = true;  ui.pidY = false; }
                    else if (ax == 'y' || ax == 'Y') { ui.pidY = true;  ui.pidX = false; }
                    else { bool on = !(ui.pidX || ui.pidY); ui.pidX = ui.pidY = on; }  // 'o' sozinho
                    // zera os integrais ao (re)ligar pra nao aplicar windup velho
                    g.controladorX.resetIntegral(); g.controladorY.resetIntegral();
                    if (!ui.pidX && !ui.pidY) g.driver.neutro();
                    printf("PID -> eixo X:%s  eixo Y:%s\n", ui.pidX ? "ON" : "off", ui.pidY ? "ON" : "off");
                } else if (cmd == 'z' || cmd == 'Z') {
                    char sub = (ui.linepos > 1) ? ui.linebuf[1] : '\0';
                    float val = (ui.linepos > 2) ? (float)atof(ui.linebuf + 2) : 0.0f;
                    if      (sub == 'x' || sub == 'X') g.driver.ajustaNeutroX(val);
                    else if (sub == 'y' || sub == 'Y') g.driver.ajustaNeutroY(val);
                    else if (sub == 's' || sub == 'S') g.driver.salvaNeutro();
                    else if (sub == '0') g.driver.resetNeutro();
                    else printf("Comando z invalido. Use zx<n>, zy<n>, zs, z0\n");
                } else if (cmd == 'u' || cmd == 'U') {
                    char sub = (ui.linepos > 1) ? ui.linebuf[1] : '\0';
                    float val = (ui.linepos > 2) ? (float)atof(ui.linebuf + 2) : 0.0f;
                    if      (sub == 'x' || sub == 'X') g.visao.ajustaCentroPx(val, 0.0f);
                    else if (sub == 'y' || sub == 'Y') g.visao.ajustaCentroPx(0.0f, val);
                    else if (sub == 's' || sub == 'S') g.visao.salvaCentroPx();
                    else if (sub == '0') g.visao.resetCentroPx();
                    else printf("Comando u invalido. Use ux<n>, uy<n>, us, u0\n");
                } else if (cmd == '0') {
                    char sub = (ui.linepos > 1) ? ui.linebuf[1] : '\0';
                    if (sub == 'x' || sub == 'X') {
                        float val = (ui.linepos > 2) ? (float)atof(ui.linebuf + 2) : 0.0f;
                        g.driver.ajustaNeutroX(val);
                    } else if (sub == 'y' || sub == 'Y') {
                        float val = (ui.linepos > 2) ? (float)atof(ui.linebuf + 2) : 0.0f;
                        g.driver.ajustaNeutroY(val);
                    } else if (sub == 'c' || sub == 'C') {
                        char eixo = (ui.linepos > 2) ? ui.linebuf[2] : '\0';
                        float val = (ui.linepos > 3) ? (float)atof(ui.linebuf + 3) * 0.1f : 0.0f;
                        if      (eixo == 'x' || eixo == 'X') g.visao.ajustaCentroCm(val, 0.0f);
                        else if (eixo == 'y' || eixo == 'Y') g.visao.ajustaCentroCm(0.0f, val);
                        else if (eixo == 's' || eixo == 'S') g.visao.salvaCentroPx();
                        else if (eixo == '0') g.visao.resetCentroPx();
                        else printf("Comando 0c invalido. Use 0cx<n>, 0cy<n>, 0cs, 0c0\n");
                    } else if (sub == 's' || sub == 'S') {
                        g.driver.salvaNeutro();
                        g.visao.salvaCentroPx();
                    } else if (sub == '0') {
                        g.driver.resetNeutro();
                        g.visao.resetCentroPx();
                    } else {
                        printf("Comando 0 invalido. Use 0x<n>, 0y<n>, 0cx<n>, 0cy<n>, 0s, 00\n");
                    }
                } else {
                    float val = (ui.linepos > 1) ? (float)atof(ui.linebuf + 1) : 0.0f;
                    ui.pidX = ui.pidY = false; ui.dancaX = ui.dancaY = false; ui.manualAtivo = true;
                    if (cmd == 'x' || cmd == 'X') { ui.manualX = val; printf("Servo X manual sinal=%.1f (neutro=%.1f %s)\n", ui.manualX, g.driver.neutroX(), SERVO_X_INVERTIDO ? "inv" : ""); }
                    else                          { ui.manualY = val; printf("Servo Y manual sinal=%.1f (neutro=%.1f)\n", ui.manualY, g.driver.neutroY()); }
                }
                ui.capturando = false; ui.linepos = 0;
            } else if (ui.linepos < 15) {
                ui.linebuf[ui.linepos++] = (char)c;
            }
            continue;
        }
        if (c == 'x' || c == 'X' || c == 'y' || c == 'Y' || c == 'o' || c == 'O' ||
            c == 'z' || c == 'Z' || c == 'u' || c == 'U' || c == '0') {
            ui.capturando = true; ui.linepos = 0; ui.linebuf[ui.linepos++] = (char)c; continue;
        }
        // --- danca (gangorra) nao-bloqueante e neutro ---
        if (c == 'w' || c == 'W') {
            ui.dancaX = !ui.dancaX; ui.pidX = ui.pidY = false; ui.manualAtivo = false;
            if (!ui.dancaX && !ui.dancaY) { ui.danceAmp = 0.0f; g.driver.neutro(); }
            printf("Danca eixo X %s\n", ui.dancaX ? "ON" : "OFF"); continue;
        }
        if (c == 'b' || c == 'B') {
            ui.dancaY = !ui.dancaY; ui.pidX = ui.pidY = false; ui.manualAtivo = false;
            if (!ui.dancaX && !ui.dancaY) { ui.danceAmp = 0.0f; g.driver.neutro(); }
            printf("Danca eixo Y %s\n", ui.dancaY ? "ON" : "OFF"); continue;
        }
        if (c == 'n' || c == 'N') {
            ui.manualAtivo = false; ui.dancaX = ui.dancaY = false; ui.pidX = ui.pidY = false;
            ui.danceAmp = 0.0f; g.driver.neutro();
            printf("Neutro (parou manual/danca/PID)\n"); continue;
        }
        if (c == 'f' || c == 'F') { printf("Frame anotado solicitado.\n"); g.visao.solicitaDebug(); }
        else if (c == 'g') { g.visao.capturaFundo(); }
        else if (c == 'G') { g.visao.limpaFundo(); }
        else if (c == 'c' || c == 'C') { g.visao.calibraCamera(); }
        else if (c == 'm' || c == 'M') { printf("Calibrando geometria da mesa...\n"); g.visao.calibraMesaGeometrica(); }
        else if (c == 'e' || c == 'E') { ui.sobel = !ui.sobel; g.visao.debugSobel(ui.sobel); printf("Sobel %s\n", ui.sobel ? "ON" : "OFF"); }
        else if (c == 'h' || c == 'H') { ui.mascara = !ui.mascara; g.visao.debugMascara(ui.mascara); printf("Mascara %s\n", ui.mascara ? "ON" : "OFF"); }
        else if (c == 'r' || c == 'R') { g.visao.resetFiltro(); }
        else if (c == 't') { ui.telemetria = !ui.telemetria; printf("Telemetria %s\n", ui.telemetria ? "ON" : "OFF"); }
        else if (c == 'T') { bool sw = g.driver.trocaEixos(); printf("Eixos dos servos: %s\n", sw ? "TROCADOS" : "normais"); }
        else if (c == '+') { g.visao.ajustaExposicao(+CAM_AEC_STEP); }
        else if (c == '-') { g.visao.ajustaExposicao(-CAM_AEC_STEP); }
        else if (c == '.') { g.visao.ajustaGanho(+1); }
        else if (c == ',') { g.visao.ajustaGanho(-1); }
        else if (c == 'a' || c == 'A') { ui.autoSens = !ui.autoSens; g.visao.autoSensor(ui.autoSens); }
        else if (c == 'k' || c == 'K') { g.visao.ajustaClock(-1); }
        else if (c == 'l' || c == 'L') { g.visao.ajustaClock(+1); }
        else if (c == 'q' || c == 'Q') {
            g.visao.imprimeSensor();
            g.visao.imprimeCentroPx();
            printf("Neutro servos: X=%.1f Y=%.1f\n", g.driver.neutroX(), g.driver.neutroY());
            printf("Ganhos atuais PID: Kp=%.1f Ki=%.2f Kd=%.2f\n", g.controladorX.kp, g.controladorX.ki, g.controladorX.kd);
        }
        else if (c == 'I') {
            g.controladorX.ki += 0.1f; g.controladorY.ki += 0.1f;
            printf("Ganhos: Kp=%.1f Ki=%.2f Kd=%.2f\n", g.controladorX.kp, g.controladorX.ki, g.controladorX.kd);
        }
        else if (c == 'i') {
            g.controladorX.ki -= 0.1f; g.controladorY.ki -= 0.1f;
            if (g.controladorX.ki < 0.0f) g.controladorX.ki = 0.0f;
            if (g.controladorY.ki < 0.0f) g.controladorY.ki = 0.0f;
            printf("Ganhos: Kp=%.1f Ki=%.2f Kd=%.2f\n", g.controladorX.kp, g.controladorX.ki, g.controladorX.kd);
        }
        else if (c == 'p') {
            g.controladorX.kp -= 1.0f; g.controladorY.kp -= 1.0f;
            if (g.controladorX.kp < 0.0f) g.controladorX.kp = 0.0f;
            if (g.controladorY.kp < 0.0f) g.controladorY.kp = 0.0f;
            printf("Ganhos: Kp=%.1f Ki=%.2f Kd=%.2f\n", g.controladorX.kp, g.controladorX.ki, g.controladorX.kd);
        }
        else if (c == 'P') {
            g.controladorX.kp += 1.0f; g.controladorY.kp += 1.0f;
            printf("Ganhos: Kp=%.1f Ki=%.2f Kd=%.2f\n", g.controladorX.kp, g.controladorX.ki, g.controladorX.kd);
        }
        else if (c == 'd') {
            g.controladorX.kd -= 0.1f; g.controladorY.kd -= 0.1f;
            if (g.controladorX.kd < 0.0f) g.controladorX.kd = 0.0f;
            if (g.controladorY.kd < 0.0f) g.controladorY.kd = 0.0f;
            printf("Ganhos: Kp=%.1f Ki=%.2f Kd=%.2f\n", g.controladorX.kp, g.controladorX.ki, g.controladorX.kd);
        }
        else if (c == 'D') {
            g.controladorX.kd += 0.1f; g.controladorY.kd += 0.1f;
            printf("Ganhos: Kp=%.1f Ki=%.2f Kd=%.2f\n", g.controladorX.kp, g.controladorX.ki, g.controladorX.kd);
        }
        else if (c == 'v' || c == 'V') {
            g.driver.debug();
        }
        else if (c == 's') {
            printf("Iniciando varredura rapida dos servos...\n");
            const float seq[] = { -10, 0, 10, 0 };
            for (int i = 0; i < 4; i++) {
                printf("Servos -> %.1f graus\n", seq[i]);
                g.driver.enviaCorrecaoX(seq[i]);
                g.driver.enviaCorrecaoY(seq[i]);
                vTaskDelay(pdMS_TO_TICKS(1000));
            }
            printf("Varredura concluida. Neutro.\n");
        }
        else if (c == '?') { comandos_ajuda(g); }
    }
}
