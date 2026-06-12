# Mesa Balanceadora PID — Plano do Projeto

Mesa (prato) inclinável em 2 eixos com uma bola em cima. Uma **ESP32-CAM (WROVER,
OV2640)** olha de cima, acha a bola por cor, calcula a posição (origem no **centro
da imagem = (0,0)**) e dois controladores **PID (PD com derivada filtrada)** inclinam
o prato via **PCA9685 + 2 servos** para levar a bola até um alvo. **Tudo roda na
própria ESP32** (sem PC).

---

## 1. Hardware e ligações

**Componentes:** ESP32-CAM WROVER (gravador integrado) · PCA9685 · 2 servos digitais
LewanSoul 20 kg · fonte externa 5 V (≥ 4–5 A).

| De | Para |
|---|---|
| Fonte 5 V `+` / `−` | `V+` / `GND` (terminais de parafuso) do PCA9685 |
| ESP32 `3V3` / `GND` | `VCC` / `GND` (lógica) do PCA9685 |
| ESP32 `GPIO14` (SDA) / `GPIO15` (SCL) | `SDA` / `SCL` do PCA9685 |
| Servo X | canal **0** do PCA9685 |
| Servo Y | canal **1** do PCA9685 |
| Cabo flat da câmera | conector da OV2640 na placa |

**Regras de ouro:**
- **GND comum** entre fonte, PCA9685 e ESP32 (senão o PWM não tem referência).
- Servos **só** no `V+` da fonte externa — **nunca** na ESP32.
- A ESP32-CAM **não tem slot de MicroSD** nesta placa → "gravar vídeo" = transmitir
  para o PC/navegador e gravar de lá (ver Etapa 1).
- Pinos de I2C (14/15) são os GPIO livres da ESP32-CAM; ajuste em `config.h` se precisar.

---

## 2. Arquitetura de software (dual-core / FreeRTOS)

```
ESP32 (FreeRTOS)
 Núcleo 1 (tempo real)                    Núcleo 0 (rede)
  tarefaVisao ──fila(Medicao)──► tarefaControle      tarefaComms (Bluetooth)
  captura+detecta                2x PID → fila de       botão / setpoint /
                                 eventos → PCA9685      ganhos / telemetria
```

Comunicação entre tarefas por **fila do FreeRTOS** (nunca variável global crua).
A atuação é agendada na sua **fila de eventos** (corrige em +20 ms), preservando o
padrão escalonador da sua `classes.c`.

### Estrutura de pastas
```
platformio.ini          # env esp32cam, framework = espidf
sdkconfig.defaults      # PSRAM, 240 MHz, tick 1 ms
CMakeLists.txt
include/
  config.h              # TODOS os parâmetros (pinos, cor, PID, servo, ROI...)
  tipos.h               # enums (estados/eventos/ações) e structs (Medicao, DadosEvento)
src/
  CMakeLists.txt
  idf_component.yml     # dependência esp32-camera (usada a partir da Etapa 1)
  main.cpp              # app_main: cria filas e tarefas (pinned to core)
  gerenciador.{h,cpp}   # amarra visão + 2 PIDs + driver + bluetooth + fila + estados
  visao.{h,cpp}         # OV2640 + detecção da bola            [NOVO]
  controle_pid.{h,cpp}  # ControladorPID + FilteredDerivative  (do seu controlePID.m)
  driver_atuacao.{h,cpp}# PCA9685 → servos
  bluetooth.{h,cpp}     # BLE: botão/setpoint/ganhos/telemetria
  fila_eventos.{h,cpp}  # escalonador de atuação (sua filaDeEventos)
  maquina_estados.{h,cpp}# matriz de transição (sua maquinaEstados.c)
```

> Os arquivos antigos na raiz (`classes.c`, `main.c`, `maquinaEstados.c`,
> `controlePID.m`, `untitled.m`) são a **referência** do projeto. O código novo fica
> em `src/`; o PlatformIO/ESP-IDF só compila o que está em `src/CMakeLists.txt`,
> então eles não atrapalham o build (podem ser arquivados depois).

### Mapa: seu código → este projeto
| Seu arquivo | Vira |
|---|---|
| `controlePID.m` (PD + filtro Td/N) | `controle_pid.*` (Kp=200, Kd=1, τ=0.01, dt=0.02) |
| `classes.c` → `controladorPID`/`FilteredDerivative` | `controle_pid.*` |
| `classes.c` → `filaDeEventos` | `fila_eventos.*` |
| `classes.c` → `driverDeAtuacao` | `driver_atuacao.*` |
| `classes.c` → `Bluetooth` | `bluetooth.*` (sem a posição da bola — agora vem da visão) |
| `classes.c` → `moduloGerenciador` | `gerenciador.*` |
| `maquinaEstados.c` | `maquina_estados.*` |

---

## 3. Roteiro de implementação (do simples ao avançado)

> Cada etapa é testável sozinha. Só passa pra próxima quando a anterior funciona.

### ✅ Etapa 0 — Estrutura e arquitetura *(FEITO)*
Esqueleto PlatformIO + ESP-IDF, `config.h` central, módulos preservando seu design,
tarefas FreeRTOS criadas (corpos com `TODO`). Compila e roda (sem fazer nada útil ainda).

### ✅ Etapa 1 — Câmera funcionando + FPS *(FEITO)*
- Pinos corrigidos para **ESP-WROVER-KIT** (XCLK=21, sem PWDN, D0–D3 próprios).
- Dois modos em `config.h` (`MODO_CAMERA`): `0` = streaming WiFi (navegador),
  `1` = teste sem WiFi (FPS de captura + foto pela serial via `tools/recebe_foto.py`).
- WiFi power-save desligado (`WIFI_PS_NONE`) — subiu o stream de ~3 para ~40–50 fps.
- **Resultado:** ~25 fps estáveis em VGA capturando local (sem WiFi); câmera OK.

### ⬜ Etapa 2 — Servos funcionando (PCA9685)
- Iniciar I2C + PCA9685 (50 Hz), mover os servos em ângulos fixos.
- Achar o **neutro** (mesa plana = 90°) e os **limites mecânicos** (ajustar
  `SERVO_MIN/MAX/NEUTRO` em `config.h`).
- Entregável: comandar os dois servos de forma controlada e segura.

### ✅ Etapa 3 — Reconhecer a mesa *(FEITO)*
- ROI da mesa definida por 4 cantos calibrados (`MESA_EXT_*` / `MESA_ROI_*`),
  com as retas extrapoladas quando a mesa não cabe inteira no frame.
- Origem (0,0) no centro, via **homografia 3×3** (corrige perspectiva).
- LUT de varredura por linha pré-computada.

### ✅ Etapa 4 — Reconhecer a bola e achar as coordenadas *(FEITO — ver [docs/VISAO.md](docs/VISAO.md))*
- Pipeline reescrito e otimizado: câmera **determinística** (exposição/ganho/AWB
  fixos + calibração na NVS) → referência local (grade 4×4 **ou** subtração de
  fundo) → **componentes conexos** (run-length + union-find) → escolha do blob
  por **score** → **centróide ponderado** sub-pixel → homografia px→cm →
  **filtro α-β** (posição suave + velocidade + gating anti-teleporte).
- Debug visual corrigido (imagem original + overlays; Sobel opcional) e
  telemetria CSV. Comandos na serial: `f g G c e r t ?`.
- Entregável: `Medicao { x, y, vx, vy, achou, dt }` confiável saindo de
  `visao.detecta()`.

### ⬜ Etapa 5 — Fechar a malha (PID equilibrando)
- Ligar visão → `gerenciador.calculaAcaoControle()` → fila de eventos → servos.
- Tunar **P → D** (o `I` quase não é usado nesse sistema).
- Entregável: a bola se estabiliza no centro.

---

## 4. Próximos passos (refinamentos)

**Já implementados na visão** (detalhes em [docs/VISAO.md](docs/VISAO.md)):
- ✅ **ROI tracking** (`ROI_FRACAO` / `ROI_PERDE_FRAMES`).
- ✅ **Filtro α-β** → posição suave + **velocidade** estimada (pronta para virar o
  termo D do PID) + gating anti-teleporte.
- ✅ **Câmera determinística** com calibração de exposição/ganho salva na **NVS**.
- ✅ **Subtração de fundo** da mesa vazia (alternativa à grade de iluminação).
- ✅ **Homografia** para px→cm (corrige perspectiva).

Ainda pendentes:
- **Predição centrando a janela** onde a bola *vai estar* (hoje a janela segue a
  última posição; falta usar a velocidade do α-β para projetar em pixels).
- **Compensar a inclinação da mesa** na projeção (a câmera vê a bola com o prato
  inclinado; corrigir com o ângulo comandado). Refinamento de 2ª ordem.
- **Auto-calibração de cor** (limiares de cor na NVS), se for usar bola colorida.
- **Bluetooth (BLE):** botão liga/desliga, setpoint, ganhos e telemetria no celular.
- **Parâmetros do PID ajustáveis em runtime** (serial/BT) — *parte de controle*.

---

## 5. Notas de referência

- **Detecção sem OpenCV** (color blob / centróide na própria ESP32): viável e
  documentado (eloquentarduino, element14).
- **FPS:** ESP32-CAM atinge ~37–40 FPS em boas condições; RGB limitado a ≤ 320×240 por
  RAM. QQVGA a ~30 Hz já é folgado para o controle (a física da bola é de poucos Hz).
- **Dual-core:** `xTaskCreatePinnedToCore`, filas FreeRTOS, `vTaskDelayUntil` para
  período fixo. Rede/BT no núcleo 0; visão+controle no núcleo 1.
- **Modelo da planta** (`controlePID.m`): `G = −m·g·d / (L·(J/R²+m)·s²)` (duplo
  integrador); controlador `H = Kp·(1 + Td·s/((Td/N)·s + 1))` (PD com derivada filtrada).
- Projetos de referência: yuxiangdai/OpenCV-Ball-Balancer, TianxingWu/Ball-and-Plate-System,
  giusenso/Ball-Balancing-PID-System.
