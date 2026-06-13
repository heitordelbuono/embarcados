# Arquitetura

## Visão geral

Um único firmware de produção (`src/`) roda a mesa: câmera → detecção da bola →
PID → servos, com uma interface no celular por WiFi. Os testes de bring-up de
hardware ficam separados em `apps/` e não entram no build normal.

```text
ESP32 (FreeRTOS)
 CORE 1 (tempo real)                         CORE 0 (rede)
  tarefa_principal (main.cpp)                 wifi_controle.cpp
   |-- Visao::detecta()                        |-- AP "MesaPID"
   |-- comandos_processa()  (serial)           |-- HTTP: GET /  (canvas)
   |-- PID / danca / manual -> servos          |-- HTTP: GET /s (posicao JSON)
   |-- g.processaFila()                         '-- HTTP: GET /sp (clique = setpoint)
   '-- status a cada 5 s
```

Rodar o loop no `app_main` (core 0) junto do WiFi derrubava o FPS e estourava o
watchdog do IDLE0; por isso o loop é uma task fixada no core 1.

## Árvore de arquivos

```text
src/
  main.cpp              app_main + loop principal (core 1)
  comandos_serial.*     ajuda + parser dos comandos do teclado
  visao/
    visao.{cpp,h}       camera/sensor + geometria + segmentacao + filtro
    debug_visao.*       overlays e envio do frame anotado (PPM/base64)
    visao_interno.h     PontoF + leitura de pixel (compartilhado)
  controle/
    gerenciador.*       amarra visao + 2 PIDs + driver + fila
    controle_pid.*      ControladorPID (PD com derivada filtrada)
    driver_atuacao.*    PCA9685 -> servos
    fila_eventos.*      escalonador da atuacao (atraso de +20 ms)
  comms/
    wifi_controle.*     AP + servidor HTTP (interface no celular)
include/
  config.h              TODOS os parametros (pinos, cor, PID, servo, ROI...)
  tipos.h               struct Medicao, DadosEvento
apps/                   testes de hardware (fora do build) — ver apps/README.md
```

## Fluxo de dados

`Visao::detecta()` devolve uma `Medicao { x, y, vx, vy, achou, dt }` em cm com
origem no centro da mesa. O loop a usa para (a) atualizar a posição publicada na
web (`g_pos_web`), (b) ler o setpoint vindo do clique no celular (`g_setpoint_*`)
e (c) rodar o PID, que agenda a correção na `FilaDeEventos`; `processaFila()`
aplica a correção vencida nos servos via `DriverAtuacao`.

## Pipeline da visão (resumo)

Captura JPEG QQVGA → decodifica on-chip para RGB565 → referência local (grade de
iluminação 4×4 **ou** fundo da mesa vazia) → candidatos por brilho → componentes
conexos (run-length + union-find) → escolhe o blob por score → centróide
ponderado sub-pixel → homografia px→cm → filtro α-β (posição + velocidade).
Detalhes em [VISAO.md](VISAO.md).

## Nota histórica: JPEG "rosa"

`jpg2rgb565()` (do componente `esp32-camera`) grava RGB565 em little-endian,
enquanto o resto do código lê big-endian (igual ao frame buffer da câmera). Sem
corrigir, o debug em JPEG sai com canais trocados (rosa/magenta). A correção está
em `decodifica_jpeg()` (visao.cpp): troca os bytes uma vez após o decode.

## Comandos de campo mais importantes (serial)

```text
g       captura fundo da mesa vazia para a deteccao
f       manda frame anotado pela serial
m       calibra geometria/homografia da mesa
c       calibra a camera (auto -> congela -> salva NVS)
q       imprime sensor, centro, neutro e PID
o/ox/oy liga PID nos dois eixos / so X / so Y
P p D d ajusta Kp (+/-1) e Kd (+/-0.1)
0x1     soma 1 grau no zero do servo X      |  0y-1  tira 1 grau do servo Y
0cx1    move centro X em +0.1 cm            |  0cy-1 move centro Y em -0.1 cm
0s      salva centro e zero dos servos      |  00    reseta ambos
```
