# 📋 Relatório Técnico — Mesa Balanceadora PID

> Documento explicativo do estado **atual** do projeto, escrito para alguém que
> nunca viu o sistema nem conhece o assunto. Vai do conceito geral até cada
> detalhe técnico de câmera, visão, controle, atuação e rede.

---

## Índice

1. [O que é o projeto](#1-o-que-é-o-projeto)
2. [Hardware](#2-hardware)
3. [Arquitetura: dois núcleos em paralelo](#3-arquitetura-dois-núcleos-em-paralelo)
4. [Diagrama geral do fluxo](#4-diagrama-geral-do-fluxo)
5. [A câmera — captura](#5-a-câmera--captura)
6. [O reconhecimento — achar a bola](#6-o-reconhecimento--achar-a-bola)
7. [O PID — a decisão de controle](#7-o-pid--a-decisão-de-controle)
8. [Os ângulos — atuação nos servos](#8-os-ângulos--atuação-nos-servos)
9. [O WiFi e a conexão](#9-o-wifi-e-a-conexão)
10. [Robustez da conexão (o que foi resolvido)](#10-robustez-da-conexão-o-que-foi-resolvido)
11. [Tabela de parâmetros atuais](#11-tabela-de-parâmetros-atuais)
12. [Glossário](#12-glossário)

---

## 1. O que é o projeto

Imagine uma **bandeja plana** com uma **bolinha** em cima. Inclinou a bandeja, a
bola rola. O objetivo é fazer a bandeja **se equilibrar sozinha** para manter a
bola num ponto escolhido (o centro, ou qualquer ponto que você tocar na tela do
celular). Empurrou a bola? Ela "volta" para o lugar.

Para isso o sistema responde, **~25 vezes por segundo**, a três perguntas:

1. **Onde está a bola?** → uma câmera olha de cima e a localiza.
2. **Para onde inclinar para trazer a bola ao alvo?** → o cálculo **PID**.
3. **Como inclinar?** → dois **servomotores** movem os eixos X e Y da mesa.

Isso é um **sistema de controle em malha fechada**: medir → decidir → agir →
medir de novo, num ciclo contínuo. Mesmo princípio de um drone se equilibrando.

O cérebro é um **ESP32**: um microcontrolador barato, com dois núcleos e WiFi
embutido, que faz **tudo ao mesmo tempo** — processa a imagem, calcula o
controle, comanda os motores e ainda serve uma página web para o celular.

---

## 2. Hardware

| Peça | Função |
|------|--------|
| **ESP32-WROVER** | Computador central. 2 núcleos @ 240 MHz, 8 MB de PSRAM (memória extra para a imagem). |
| **Câmera OV2640** | "Olho" do sistema; olha a mesa de cima. |
| **2 servomotores** | "Músculos"; inclinam a mesa nos eixos X (esquerda/direita) e Y (frente/trás). |
| **Driver PCA9685** | "Tradutor" entre ESP32 e servos. O ESP fala por I²C (2 fios) e ele gera os pulsos precisos dos servos. |
| **Bola + mesa** | Bola branca de ~4 cm sobre mesa cinza de **19×19 cm**. O contraste claro/escuro é o que permite enxergá-la. |

Coordenadas usadas no software: origem **(0,0) no centro da mesa**, eixos indo de
aproximadamente **−9,5 cm a +9,5 cm** em cada direção.

---

## 3. Arquitetura: dois núcleos em paralelo

O ESP32 tem **dois núcleos**, e o trabalho é dividido de propósito:

- **Núcleo 1** → o **loop de controle** (câmera → detecção → PID → servos), sem
  parar, o mais rápido possível (~25 quadros/s).
- **Núcleo 0** → o **WiFi e a página web**.

**Por que separar?** O WiFi é "barulhento": interrompe o processador o tempo todo
para tratar pacotes. No mesmo núcleo do controle, isso faria a câmera engasgar e a
mesa oscilar. Isolando o controle no núcleo 1, ele roda liso independentemente da
rede.

Os dois núcleos compartilham poucos dados (posição da bola, ponto-alvo). Como
ambos podem acessá-los ao mesmo tempo, esses dados são protegidos por um
**"cadeado" (spinlock)** que impede leitura de um valor pela metade.

```
        NÚCLEO 1 (tempo real)                 NÚCLEO 0 (rede)
   ┌───────────────────────────┐        ┌────────────────────────┐
   │  câmera → visão → PID →    │        │  WiFi (Access Point)   │
   │  servos, repetindo ~25x/s  │        │  servidor HTTP + WS    │
   └───────────┬───────────────┘        └───────────┬────────────┘
               │                                     │
               │   estado compartilhado (protegido)  │
               └──────────────►  [ x, y, achou, fps, │
                                   setpoint X/Y ]  ◄──┘
```

> 🔑 **O controle é 100% local.** O WiFi serve só para você ver a bola e escolher o
> alvo. Se o celular cair, a mesa continua equilibrando normalmente.

---

## 4. Diagrama geral do fluxo

O ciclo completo, repetido continuamente no núcleo 1:

```
   ┌─────────────┐
   │   CÂMERA    │  tira foto QQVGA 160×120 em JPEG
   │   OV2640    │
   └──────┬──────┘
          │  buffer JPEG
          ▼
   ┌─────────────┐
   │  DECODIFICA │  JPEG ─► RGB565 (pixels crus na PSRAM)
   └──────┬──────┘
          │
          ▼
   ┌─────────────────────────────────────────────┐
   │            RECONHECIMENTO (visão)            │
   │  1. recorta a ROI (área da mesa)             │
   │  2. marca pixels claros vs. fundo local      │
   │  3. agrupa em manchas (run-length+union-find)│
   │  4. filtra por tamanho / forma / densidade   │
   │  5. centroide sub-pixel (centro de massa)    │
   └──────┬──────────────────────────────────────┘
          │  centro em PIXELS
          ▼
   ┌─────────────┐
   │ HOMOGRAFIA  │  pixels ─► centímetros (corrige a perspectiva)
   └──────┬──────┘
          │  posição crua (cm)
          ▼
   ┌─────────────┐
   │ FILTRO α-β  │  suaviza a posição + estima a VELOCIDADE (cm/s)
   └──────┬──────┘
          │  x, y, vx, vy, achou
          ├───────────────────────────────►  (publica para o WiFi/tela)
          ▼
   ┌─────────────────────────────┐
   │   PID  (um para X, um p/ Y)  │  erro = alvo − posição
   │   saída = Kp·e + Ki·∫e + Kd·ė│
   └──────┬──────────────────────┘
          │  sinal de correção
          ▼
   ┌─────────────────────────────┐
   │  ESCALA ×0,1  →  SATURA ±40° │  limita a inclinação
   └──────┬──────────────────────┘
          │  graus de correção
          ▼
   ┌─────────────────────────────┐
   │  ÂNGULO = neutro ± correção  │  X≈91°, Y≈97° de base
   └──────┬──────────────────────┘
          │  ângulo final (0–180° de segurança)
          ▼
   ┌─────────────┐
   │   PCA9685   │  ângulo ─► pulso (500–2500 µs) @ 50 Hz
   └──────┬──────┘
          ▼
   ┌─────────────┐
   │   SERVOS    │  inclinam a mesa
   └──────┬──────┘
          ▼
     A BOLA ROLA um pouco em direção ao alvo...
          │
          └──────────►  e o ciclo recomeça (volta para a CÂMERA)
```

Em paralelo, no núcleo 0:

```
   CELULAR  ◄───── WebSocket (20x/s) ─────►  ESP32
   - desenha a bola e o alvo               - lê o estado compartilhado
   - toque na tela = novo alvo  ──────────► - atualiza o setpoint do PID
```

---

## 5. A câmera — captura

A cada ciclo a câmera entrega um quadro. As decisões importantes:

- **Resolução baixa de propósito: QQVGA (160×120 = 19.200 pixels).** Menos pixels
  = processamento mais rápido = mais quadros por segundo (objetivo: 20+). Alta
  resolução tornaria o controle lento demais para equilibrar.
- **Entrega em JPEG** (sai mais rápido do sensor, com o clock cheio) e o ESP32
  **descomprime** cada quadro para **RGB565** (cor crua) na PSRAM, para analisar
  pixel a pixel.
- **`grab_mode = LATEST`**: sempre pega o quadro **mais recente**, descartando
  atrasados. Em controle, imagem velha é pior que inútil.
- **Double buffer (`fb_count = 2`)**: enquanto um quadro é processado, o próximo já
  está sendo capturado.
- **Clock do sensor: 20 MHz.**
- **Exposição/ganho/balanço de branco podem ser travados** (comando de
  calibração, ~1,2 s no automático e depois congela). Câmera no automático muda o
  brilho sozinha conforme a luz, atrapalha a detecção e derruba o FPS. Travada, a
  imagem fica previsível.

A captura em si é rápida (~0,3 ms); o custo está no **processamento** (~35 ms por
quadro), onde mora a inteligência.

---

## 6. O reconhecimento — achar a bola

Objetivo: a partir dos pixels, obter **as coordenadas do centro da bola** em cm.

### 6.1. Região de interesse (ROI)
A bola só é procurada **dentro do quadrilátero da mesa**, definido por 4 cantos
calibrados (em resolução de calibração 640×480, reescalados para 160×120). Tudo
fora — chão, mãos, reflexos — é ignorado. Acelera e evita falsos positivos.

### 6.2. Quais pixels são "candidatos a bola"
A bola é **branca sobre fundo escuro**. Um pixel é candidato se for **mais claro
que a referência local** por uma margem mínima (`BOLA_DELTA_REF = 10`) **e** acima
de um piso absoluto de brilho (`BOLA_Y_MIN = 80`).

A referência **não é fixa**: a mesa pode ter um canto mais claro que o outro,
então usa-se uma **grade 4×4 de iluminação** (cada célula tem seu próprio nível de
fundo). Opcionalmente, uma foto da **mesa vazia** é subtraída de cada quadro
(comando de captura de fundo), o que torna a detecção imune a iluminação desigual.

### 6.3. Agrupar os pixels (rotulação)
Os candidatos espalhados são agrupados em **manchas conectadas** (blobs) com uma
técnica eficiente (**run-length + union-find**: junta segmentos vizinhos que se
tocam). Há um limite de segurança de segmentos por quadro (`VISAO_MAX_RUNS = 3072`);
se estourar (muito reflexo), o quadro é tratado como "não achei".

> ⚡ Para acelerar, a varredura pode pular pixels (`VISAO_SCAN_STEP = 2`, analisa
> ~1/4 dos pixels). Depois, no refino, faz uma 2ª passada pixel a pixel só dentro
> da caixa da bola.

### 6.4. Qual mancha é a bola (filtros de validação)
Nem toda mancha branca é a bola. São rejeitadas:
- **Tamanho** fora da faixa (pontinho de reflexo ou mancha grande de luz);
- **Redondeza**: a caixa que envolve a mancha precisa ser ~quadrada;
- **Densidade**: a mancha precisa preencher bem a caixa (bola enche; risco fino não).

### 6.5. Centro com precisão sub-pixel
Achada a bola, calcula-se o **centroide ponderado pelo brilho** — o centro de
massa. Isso dá precisão **melhor que um pixel** (o centro fica "entre" os pixels),
essencial para um controle suave.

### 6.6. De pixel para centímetros — homografia
O centro vem em pixels, mas o controle precisa de **cm com origem no centro da
mesa**. Como a câmera olha meio de lado, a mesa aparece como um **quadrilátero
torto**, não um quadrado. Uma regra de três não bastaria. Usa-se uma
**homografia**: uma transformação que "desentorta" a perspectiva e mapeia qualquer
pixel da mesa para sua posição real em cm. Saída típica: `x = +2,3 cm, y = −0,6 cm`.

### 6.7. Filtro alfa-beta — suavização + velocidade
A medição crua treme de quadro a quadro (ruído). O **filtro alfa-beta**:
- **Suaviza** a posição (ganho α = 0,55: quanto maior, mais "segue" a medição crua);
- **Estima a velocidade** da bola (ganho β = 0,12), em cm/s — útil para antecipar.

Funciona em dois passos: **prevê** onde a bola estaria (posição anterior + velocidade ×
tempo) e **corrige** essa previsão com a medição nova. Se a bola some por alguns
quadros, o filtro é reiniciado e o sistema marca `achou = false` em vez de inventar.

---

## 7. O PID — a decisão de controle

Sabendo onde a bola está, **quanto** inclinar para levá-la ao alvo? Decide o
**controlador PID**, rodado **separadamente para cada eixo (X e Y)**.

O PID olha o **erro = (alvo − posição atual)** e soma três contribuições:

| Termo | O que faz | Analogia |
|-------|-----------|----------|
| **P** — Proporcional | Reage ao **erro atual**. Mais longe → correção mais forte. | "Bola longe → inclina bastante." |
| **I** — Integral | Acumula o erro no tempo. Corrige **desvios persistentes** (ex.: mesa levemente torta). | "Faz tempo que insiste à esquerda → vou compensando mais." |
| **D** — Derivativo | Reage à **velocidade** do erro. Freia antes de passar do ponto. | "Vindo rápido → segura, senão passa direto." |

Fórmula: **saída = Kp·erro + Ki·(erro acumulado) + Kd·(taxa de variação)**.

Ganhos atuais (padrão, ajustáveis ao vivo pela serial): **Kp = 9, Ki = 40,
Kd = 10**.

**Refinamentos importantes:**

- **Derivada filtrada** (constante τ = 0,01): o termo D é sensível a ruído; um
  filtro evita que ele "pire" com tremores pequenos.
- **Anti-windup** (`PID_I_MAX = 25`): se a bola fica presa contra a borda, o termo
  Integral poderia crescer sem limite e depois "explodir". O acúmulo é **limitado**,
  mantendo o controle saudável. *(Detalhe: o integral não é zerado quando a bola
  some — é um ponto de melhoria futura possível.)*
- **dt real entre quadros**: o cálculo usa o **tempo real** medido entre quadros,
  não um valor fixo. Assim, derivada e integral ficam corretos mesmo se o FPS varia.

---

## 8. Os ângulos — atuação nos servos

A saída do PID é um número abstrato. Ela vira ângulo de servo assim:

1. **Escala ×0,1** (`PID_ESCALA_SAIDA`): traz a saída para uma faixa de graus
   razoável.
2. **Saturação ±40°** (`SERVO_RANGE`): a correção é limitada a ±40° em torno do
   neutro — impede inclinações violentas.
3. **Soma ao neutro**: cada servo tem um ângulo de "mesa plana" calibrado
   (X = 91°, Y = 97°), salvo na memória não-volátil (NVS). A correção é
   somada/subtraída desse neutro.
   - Faixa de operação resultante: **X de 51° a 131°**, **Y de 57° a 137°**.
   - Há ainda um limite físico absoluto de **0–180°** como rede de proteção.
4. **Inversão de eixo**: o eixo X é invertido por configuração, pois a montagem do
   servo gira no sentido oposto ao da imagem.
5. **Conversão para pulso**: o ângulo vira um pulso entre **500 e 2500 µs**, que o
   **PCA9685** envia ao servo a **50 Hz**.

Há também uma **fila de eventos** (agendador) que pode aplicar correções com um
pequeno atraso configurável — hoje em **0 µs** (atua imediatamente).

> ⚠️ Quando a bola **não é encontrada**, o sistema **não envia correção nova** — os
> servos seguram a última posição. Esse comportamento pode ser trocado por "voltar
> ao neutro" se desejado.

---

## 9. O WiFi e a conexão

O ESP32 cria sua **própria rede WiFi** (Access Point **"MesaPID"**, aberta, canal
6). Conecte o celular nela e abra **`http://192.168.4.1`**. Não precisa de internet
nem roteador.

### O que a página mostra
Um **canvas** desenha em tempo real: a mesa, a grade, a **bola** (laranja) na
posição detectada e a **mira do alvo** (amarela). **Toque na tela** para definir um
novo alvo — a mesa leva a bola até lá.

### Como os dados trafegam — WebSocket
A comunicação usa um **WebSocket**: um "cano" aberto e permanente entre celular e
ESP32, por onde trocam mensagens minúsculas continuamente.

```
   CELULAR                                  ESP32 (núcleo 0)
   ───────                                  ────────────────
   a cada 50 ms:  {"cmd":"tick"}  ───────►  lê estado compartilhado
                  {x,y,a,f,sx,sy} ◄───────  responde a posição + alvo
   toque na tela: {"cmd":"sp",x,y} ──────►  atualiza o setpoint do PID
```

- O navegador manda `tick` **20×/s**; o ESP responde com posição, FPS e alvo.
- Tocar na tela envia o novo setpoint (`sp`) pelo mesmo cano.

**Por que WebSocket?** A versão antiga abria/fechava uma conexão de rede a cada
consulta — lento e instável. O WebSocket mantém **uma conexão viva**: muito mais
rápido e estável.

Há também **`http://192.168.4.1/health`**: diagnóstico com tempo ligado, memória
livre, número de sockets e se o WebSocket está ativo.

---

## 10. Robustez da conexão (o que foi resolvido)

A conexão caía e foi estabilizada atacando as **três camadas**. A soma das
correções é o que deixou estável:

| # | Camada | Problema | Correção |
|---|--------|----------|----------|
| 1 | **WiFi** | O celular largava o AP sozinho a cada ~10 s; lag e sockets meio-abertos. | **Desligado o power-save** (`WIFI_PS_NONE`). Por padrão o ESP "cochila" entre pacotes; isso deixava o AP errático. **Conserto decisivo.** |
| 2 | **TCP** | Um engasgo momentâneo do WiFi estourava o buffer de envio → `send error 11` (EAGAIN) → socket derrubado. | **Buffer de envio dobrado** (~3 s → ~6 s de folga de ticks). |
| 3 | **Aplicação** | Socket morria "sem avisar"; o navegador continuava num socket morto e nunca reconectava (coordenada congelava). | **Watchdog no navegador**: sem dado por 1,5 s → força reconexão; e reconecta ao voltar o foco (tela liga). Backoff de 300→600→1200→1500 ms. |
| 4 | **HTTP** | Página podia vir de cache velho; `favicon.ico` gerava 404 e gastava socket; poucos sockets de cliente. | `Cache-Control: no-store`; handler de favicon (204); `max_open_sockets = 7` (o servidor reserva 3 internamente → 4 para clientes). |

Resultado: conexão estável, atualização fluida a 20 Hz, reconexão automática
quando o celular dorme/acorda.

---

## 11. Tabela de parâmetros atuais

| Parâmetro | Valor | Onde |
|-----------|-------|------|
| Resolução de detecção | QQVGA 160×120 | câmera |
| Formato de captura | JPEG → RGB565 | câmera |
| Clock do sensor | 20 MHz | câmera |
| Buffers de câmera | 2 (double buffer) | câmera |
| Taxa de quadros típica | ~25 FPS | loop |
| Tamanho da mesa | 19 × 19 cm | físico |
| Brilho mínimo da bola | 80 (0–255) | visão |
| Contraste sobre o fundo | +10 | visão |
| Grade de iluminação | 4 × 4 | visão |
| Passo de varredura | 2 (¼ dos pixels) | visão |
| Limite de segmentos/quadro | 3072 | visão |
| Filtro α (posição) | 0,55 | filtro |
| Filtro β (velocidade) | 0,12 | filtro |
| **PID Kp** | **9,0** | controle |
| **PID Ki** | **40,0** | controle |
| **PID Kd** | **10,0** | controle |
| Filtro da derivada (τ) | 0,01 | controle |
| Anti-windup (I máx) | 25 | controle |
| Escala da saída do PID | ×0,1 | controle |
| Atraso de atuação | 0 µs (imediato) | atuação |
| Neutro servo X / Y | 91° / 97° | atuação |
| Faixa de correção | ±40° | atuação |
| Faixa efetiva X / Y | 51–131° / 57–137° | atuação |
| Limite físico do servo | 0–180° | atuação |
| Pulso do servo | 500–2500 µs @ 50 Hz | atuação |
| Rede WiFi | AP "MesaPID" (aberta, canal 6) | rede |
| Endereço | http://192.168.4.1 | rede |
| Taxa de atualização da tela | 20 Hz (tick 50 ms) | rede |
| Power-save WiFi | desligado (WIFI_PS_NONE) | rede |

> Os ganhos PID e o neutro dos servos podem ser ajustados **ao vivo** pela conexão
> serial e salvos na memória não-volátil (NVS).

---

## 12. Glossário

- **Malha fechada**: sistema que mede o resultado e se corrige continuamente.
- **PID**: a fórmula de controle (Proporcional + Integral + Derivativo).
- **Setpoint**: o ponto-alvo onde se quer a bola.
- **ROI** (*Region of Interest*): região da imagem onde se procura a bola.
- **Centroide**: o ponto central (de massa) de uma mancha de pixels.
- **Homografia**: transformação matemática que corrige a perspectiva da câmera.
- **Filtro alfa-beta**: estimador que suaviza posição e calcula velocidade.
- **Anti-windup**: trava de segurança no termo Integral do PID.
- **Servo**: motor que vai a um ângulo específico sob comando.
- **PWM / pulso**: o jeito de comandar o servo (largura do pulso = ângulo).
- **Access Point (AP)**: o ESP32 criando sua própria rede WiFi.
- **WebSocket**: canal de comunicação permanente e rápido entre celular e ESP32.
- **EAGAIN**: erro de "tente de novo" — o buffer de envio estava cheio.
- **PSRAM**: memória extra do ESP32, usada para guardar a imagem.
- **NVS**: memória não-volátil — guarda calibrações mesmo desligando.
- **Núcleo (core)**: cada uma das duas "CPUs" do ESP32.

---

*Documento gerado a partir do código-fonte atual (`src/visao/`, `src/controle/`,
`src/comms/`, `include/config.h`). Para detalhes de implementação, ver também
`docs/ARQUITETURA.md` e `docs/VISAO.md`.*
