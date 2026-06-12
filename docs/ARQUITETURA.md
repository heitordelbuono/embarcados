# Arquitetura Atual e Plano de Limpeza

## Diagrama

```text
Serial/USB
   |
   v
main.cpp (ETAPA 4)
   |-- parser de comandos
   |     |-- sensor: + - . , a c j k l q S
   |     |-- visao: f g G e r m t 0cx/0cy ux/uy
   |     |-- servo: x/y manual, 0x/0y, z*, w/b/n/T
   |     '-- PID: o/ox/oy P/p I/i D/d
   |
   |-- loop de tempo real
   |     |-- Visao::detecta()
   |     |-- ModuloGerenciador::calculaAcaoControle()
   |     |-- DriverAtuacao::enviaCorrecaoX/Y()
   |     '-- status a cada 5 s
   |
   v
gerenciador.cpp
   |-- ControladorPID X/Y
   |-- FilaDeEventos
   '-- DriverAtuacao

visao.cpp
   |-- camera/sensor OV2640
   |-- decode JPEG -> RGB565
   |-- NVS camera/centro
   |-- geometria/homografia da mesa
   |-- referencia: grade local ou fundo capturado com g
   |-- segmentacao da bola
   |-- filtro alfa-beta
   '-- debug PPM/base64/overlays
```

## Diagnostico

`main.cpp` esta inchado porque junta quatro coisas diferentes: selecao de etapa, console serial, rotina de controle em tempo real e testes/sweeps. O trecho da ETAPA 4 e o mais critico: o parser esta dentro do loop principal, entao cada comando novo aumenta o miolo que deveria ser so captura, controle e status.

`visao.cpp` tambem esta com muitas responsabilidades. Ele controla hardware da camera, faz conversao de imagem, guarda calibracao na NVS, calcula homografia, segmenta a bola, filtra a posicao e ainda gera imagem de debug. Isso funcionou para evoluir rapido, mas agora qualquer bug visual parece bug de deteccao, e qualquer ajuste de sensor fica misturado com geometria.

## JPEG rosa

A suspeita mais forte nao e exposicao. Exposicao errada deixaria a imagem clara/escura ou estourada tambem no JPEG original. O rosa apareceu no caminho em que o firmware decodifica JPEG para RGB565 e depois gera o PPM/debug.

O componente `esp32-camera` chama `esp_jpeg_decode` por `jpg2rgb565()` com `swap_color_bytes = 0`. No ESP32, o decoder ROM trabalha em RGB888 e a conversao para RGB565 grava o pixel em little-endian. O nosso `rgb565_at()` le como big-endian, igual ao frame RGB565 da camera. Resultado: bytes trocados e cores puxando para magenta/rosa.

Correcao aplicada: depois de `jpg2rgb565()`, o buffer decodificado e convertido para RGB565 big-endian uma vez. Assim `pixel_luma()`, overlays e PPM continuam usando uma unica convencao.

## O Que Da Para Tirar ou Isolar

- `main.cpp`: mover parser serial para `console_comandos.*`.
- `main.cpp`: mover `sweep_sensor()` para `tools_runtime.*` ou para dentro de uma classe de diagnostico.
- `main.cpp`: manter no `app_main()` so inicializacao, loop de frame, atuacao e status.
- `visao.cpp`: separar `visao_camera.*` para init/reinit/sensor/AEC/AGC/AWB/JPEG.
- `visao.cpp`: separar `visao_debug.*` para PPM/base64/desenho.
- `visao.cpp`: separar `visao_geometria.*` para mesa, homografia, centro manual.
- `visao.cpp`: deixar `visao.cpp` orquestrando captura -> segmentacao -> filtro -> Medicao.

## Plano Seguro

1. Congelar comportamento atual com build passando e comandos de campo documentados.
2. Extrair o parser serial da ETAPA 4 sem alterar comandos.
3. Extrair debug PPM/overlays da visao.
4. Extrair camera/sensor/decode da visao.
5. So depois disso limpar ETAPAs antigas ou codigo de teste que nao esta sendo usado.

## Comandos de Campo Mais Importantes

```text
g       captura fundo da mesa vazia para a deteccao
f       manda frame anotado
j       alterna GRAYSCALE/JPEG
q       imprime sensor, centro, neutro e PID
0x1     soma 1 grau no zero do servo X
0y-1    tira 1 grau do zero do servo Y
0cx1    move centro X em +0.1 cm
0cy-1   move centro Y em -0.1 cm
0s      salva centro e zero dos servos
00      reseta centro e zero dos servos
```
