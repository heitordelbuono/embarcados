# apps/ — testes de hardware (bring-up)

Estes arquivos **não entram no build normal**. Cada um tem o seu próprio
`app_main` e serve para isolar/validar uma parte do hardware. O firmware de
produção é `src/main.cpp` (mesa + visão + PID + interface WiFi).

Para gravar um destes testes, edite `src/CMakeLists.txt` e troque a linha
`"main.cpp"` do bloco `SRCS` pelo caminho do teste (e pelos módulos que ele
usa). Depois `pio run -t upload`. Lembre de reverter para `"main.cpp"` ao
voltar à produção.

| Arquivo | O que testa | SRCS necessários (além do teste) |
|---|---|---|
| `teste_pca9685.cpp` | I2C + PCA9685 cru (scan + varredura de pulso) | nenhum |
| `teste_servo_ledc.cpp` | servo por PWM direto (LEDC), sem PCA9685 | nenhum |
| `teste_servos.cpp` | servos via `DriverAtuacao` (neutro, limites, gangorra) | `controle/driver_atuacao.cpp` |
| `teste_camera.cpp` | câmera OV2640 (stream WiFi **ou** foto+FPS pela serial) | `../apps/wifi_stream.cpp` e/ou `../apps/camera_teste.cpp` |

`wifi_stream.cpp` e `camera_teste.cpp` são auxiliares de `teste_camera.cpp`.
O modo da câmera (stream WiFi × foto pela serial) é o `MODO_CAMERA` em
`include/config.h`.
