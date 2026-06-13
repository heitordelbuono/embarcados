# Visão — detecção da bola e cálculo de coordenadas

Documento da etapa de **imagem + coordenadas** (o controle/PID fica em outro
documento). Tudo roda na própria ESP32, sem PC. Código em
[`src/visao/visao.cpp`](../src/visao/visao.cpp) /
[`src/visao/visao.h`](../src/visao/visao.h) (render de debug em
[`src/visao/debug_visao.cpp`](../src/visao/debug_visao.cpp)); parâmetros
em [`include/config.h`](../include/config.h).

> Esta é a reescrita do pipeline antigo. Os problemas que ela resolve estão
> listados na seção [O que mudou](#o-que-mudou-e-por-quê) no fim.

---

## Pipeline (visão geral)

```
 esp_camera_fb_get (JPEG QQVGA 160x120, sensor FIXO) ─► decode RGB565
        │
        ▼
 referência local por pixel ──► grade de iluminação 4x4  (padrão)
   (quão claro é o "fundo")  └─► OU fundo da mesa vazia (comando 'g')
        │
        ▼
 candidatos:  luma - referência ≥ BOLA_DELTA_REF  E  luma ≥ BOLA_Y_MIN
        │
        ▼
 componentes conexos  (run-length + union-find, 1 passada)
        │
        ▼
 escolhe 1 blob por SCORE  (forma + densidade + proximidade da última pos)
        │
        ▼
 centróide ponderado pela intensidade (sub-pixel)  [+ refino step=1 no bbox]
        │
        ▼
 homografia 3x3  px ──► cm   (origem no centro, +X esquerda, +Y cima)
        │
        ▼
 filtro α-β:  posição suave  +  velocidade (cm/s)
        │
        ▼
 Medicao { x, y, vx, vy, achou, dt, t_us }
```

---

## Cada etapa, em detalhe

### 1. Captura — câmera determinística
`esp_camera` em **JPEG QQVGA 160×120** (clock cheio do sensor) **decodificado
on-chip para RGB565** a cada frame. O ponto-chave: **exposição, ganho e AWB são FIXADOS**
(`aplica_sensor_fixo`). Auto-exposição faz o brilho da cena flutuar quando a
bola se move → qualquer limiar fica perseguindo um alvo móvel. Fixar estabiliza
todo o resto.

- Valores padrão: `CAM_AEC_FIXO`, `CAM_AGC_FIXO`.
- **Calibração** (comando `c`): liga o auto por `CAM_CALIBRA_MS`, lê os valores
  que o sensor escolheu, congela e **salva na NVS** (`CAM_USA_NVS`). No próximo
  boot esses valores são lidos automaticamente.
- Para voltar ao automático (debug), `CAM_AUTO_AJUSTE 1`.

### 2. Referência local (o "fundo")
Um pixel só interessa se for **mais claro que a vizinhança**. A referência por
pixel vem de um de dois modos:

- **Grade de iluminação** (padrão): a ROI é dividida em `GRADE_NX × GRADE_NY`
  células; cada célula tem sua média de brilho, recalculada a cada
  `VISAO_MEDIA_INTERVALO` frames. Imune a iluminação desigual (um canto mais
  claro que o outro). Substitui o antigo "média global + delta".
- **Subtração de fundo** (comando `g`): você fotografa a **mesa vazia** e ela
  vira a referência pixel a pixel. Mais robusto. Com `FUNDO_ATUALIZA`, o fundo
  é atualizado lentamente (média móvel) **fora** da bola, absorvendo deriva de
  luz. `G` descarta o fundo e volta para a grade.

### 3. Candidatos
`candidato = (luma ≥ BOLA_Y_MIN) && (luma − referência ≥ BOLA_DELTA_REF)`.
`BOLA_Y_MIN` é um piso absoluto; `BOLA_DELTA_REF` é o contraste mínimo sobre o
fundo. A detecção é por **brilho** (bola branca sobre mesa cinza).

### 4. Componentes conexos — *o conserto mais importante*
Antes, **todos** os pixels acima do limiar viravam **um único blob**: bola +
reflexo + borda da mesa caíam no mesmo bounding box, e o centróide ia parar no
meio de tudo (posição errada). Agora:

- Durante a varredura, pixels candidatos consecutivos numa linha formam um
  **run**. Runs de linhas vizinhas que se sobrepõem são unidos via
  **union-find**. Resultado: cada mancha separada é um **componente** próprio.
- Memória limitada por `VISAO_MAX_RUNS`; se estourar (limiar frouxo demais), o
  frame é marcado `overflow` e tratado como "não achei".
- Custo O(pixels varridos), 1 passada.

### 5. Escolha do blob (SCORE)
Cada componente passa pelos filtros de plausibilidade (tamanho
`BOLA_MIN/MAX_PIXELS`, lados `BOLA_MIN/MAX_LADO`, forma ~quadrada, densidade
≥15%). Entre os que passam, escolhe-se o de maior **score**:

```
score = densidade(%) + (tracking ? 120 / (1 + dist_da_última_posição) : 0)
```

Ou seja: prefere o blob compacto **e** perto de onde a bola estava — rejeita
reflexos espúrios mesmo que momentaneamente pareçam bola.

### 6. Centróide sub-pixel
Centróide **ponderado pela intensidade** (peso = `luma − referência`): o centro
"puxa" para a parte mais brilhante da bola, com precisão sub-pixel
(`CENTROIDE_PONDERADO`). Com `VISAO_SCAN_STEP > 1`, há uma **2ª passada
`step=1` só dentro do bounding box** do blob escolhido (`REFINO_SUBPIXEL`) —
barato, porque o bbox é pequeno, e recupera a precisão perdida na varredura
esparsa.

### 7. Pixel → cm (homografia)
A conversão usa uma **homografia 3×3** calculada **uma vez** na inicialização a
partir dos 4 cantos calibrados (`MESA_EXT_*`) mapeados para o quadrado real de
`MESA_LADO_CM`. Isso corrige a **perspectiva** (trapézio) corretamente — o
método antigo usava eixos/escala médios e errava nos cantos. Em runtime é só 1
multiplicação 3×3 + 1 divisão: **mais preciso e mais barato**.

Convenção (igual ao diagrama do projeto): **origem no centro, +X para a
esquerda, +Y para cima**.

### 8. Filtro α-β (posição + velocidade)
A medição crua passa por um filtro **alfa-beta** (`FILTRO_ATIVO`), que estima
posição e velocidade com `dt` **real** (medido entre frames, não fixo):

- **Suaviza** a posição (`FILTRO_ALFA`) → menos ruído.
- Estima **velocidade** `vx, vy` em cm/s (`FILTRO_BETA`) → já pronta para o
  termo D do PID mais tarde.
- Após `ROI_PERDE_FRAMES` sem bola, o filtro reinicia (próxima detecção é aceita
  como nova).

### Tracking (ROI dinâmica)
Depois de achar a bola, a varredura do próximo frame se limita a uma janela de
`ROI_FRACAO` em volta da última posição (mais FPS, menos ruído). Após
`ROI_PERDE_FRAMES` sem achar, varre a mesa inteira de novo.

---

## Comandos (pela serial)

**Detecção / coordenada:**

| Tecla | Ação |
|---|---|
| `f` | manda o próximo frame anotado pela serial (imagem **original** + overlays) |
| `g` | captura o **fundo** (mesa vazia) → ativa subtração de fundo |
| `G` | descarta o fundo (volta para a grade de iluminação) |
| `e` | liga/desliga overlay **Sobel** no frame de debug |
| `r` | reinicia o filtro α-β |
| `t` | liga/desliga **telemetria CSV** (`t,x,y,vx,vy,achou,fps`) |
| `?` | ajuda |

**Sensor / FPS (ajuste ao vivo, sem recompilar):**

| Tecla | Ação |
|---|---|
| `+` / `-` | exposição (`aec`) ± `CAM_AEC_STEP` — **menor = mais FPS, imagem mais escura** |
| `.` / `,` | ganho (`agc`) ± 1 — clareia **sem** custar FPS, mas adiciona ruído |
| `a` | liga/desliga **AUTO** (exposição/ganho/AWB) |
| `k` / `l` | divisor de clock DVP − / + (`k` = mais rápido; pode corromper o frame) |
| `c` | **calibra a câmera** (auto → congela → salva na NVS) |
| `q` | imprime o estado do sensor (`aec`/`agc`/auto/`clkdiv`) e os ganhos do PID |

> **Como ler o FPS:** o status `[5s] fps_med=... cap=...ms proc=...ms` sai a cada
> 5 s. `cap` = captura+decode (câmera); `proc` = processamento. Se `cap ≫ proc`, o
> gargalo é a câmera (mexa em exposição/clock); se `proc` for grande, é o código.

## Modo de captura

A câmera captura sempre em **JPEG** (clock cheio do sensor → ~50 fps) e o firmware
**decodifica on-chip para RGB565** a cada frame, que segue o pipeline. Decode
~24 ms, então o efetivo fica em ~25–33 fps. *Nota: esta versão do `esp32-camera`
só expõe `jpg2rgb565`; não há decode "só-Y", então o decode não dá para encurtar
sem baixar resolução.* O sweep de FPS e o modo grayscale direto foram removidos do
firmware de produção (eram ferramentas de bring-up).

### Overlays do frame de debug
imagem original ao fundo (não mais o Sobel destruindo tudo) com: **amarelo** =
mesa + cruz na origem (0,0); **verde** = ROI; **magenta** = janela varrida;
**ciano** = pixels candidatos do blob; **vermelho** = bbox da bola; **azul** =
centróide.

---

## Roteiro de calibração (ordem recomendada)

1. **Câmera**: aponte para a mesa, rode `c` (congela exposição/ganho). Faça uma
   vez por ambiente de luz; fica salvo na NVS.
2. **Geometria**: ajuste os 8 pontos `MESA_EXT_*` / `MESA_ROI_*` em `config.h`
   (medidos numa foto VGA; o firmware escala sozinho). Valide com `f` — os
   polígonos amarelo/verde devem casar com a mesa.
3. **Fundo**: com a mesa **vazia**, rode `g`. (Opcional, mas recomendado.)
4. **Limiar**: ponha a bola, rode `f`. Ajuste `BOLA_DELTA_REF` / `BOLA_Y_MIN`
   até o ciano cobrir só a bola. Ajuste `BOLA_*_PIXELS` / `BOLA_*_LADO` ao
   tamanho real em pixels (veja `area`/`bbox` no META).
5. **Validação**: `t` para telemetria, mova a bola na mão e rode
   `tools/plot_telemetria.py` para ver a trajetória e a velocidade.

---

## Scripts (PC)

- `tools/recebe_debug_visao.py <porta> <baud> [comando]` — recebe o frame
  anotado, salva `debug_bola.ppm` **e** `debug_bola.png` (3×). O `comando`
  opcional é enviado antes (ex.: `g` captura fundo e já pede o frame).
- `tools/plot_telemetria.py <porta> <baud>` — liga a telemetria, coleta até
  Ctrl+C, salva `telemetria.csv`/`telemetria.png` (trajetória + séries).

---

## Parâmetros (resumo, em `config.h`)

| Grupo | Defines |
|---|---|
| Limiar | `BOLA_Y_MIN`, `BOLA_DELTA_REF` |
| Forma/tamanho | `BOLA_MIN/MAX_PIXELS`, `BOLA_MIN/MAX_LADO` |
| Referência | `GRADE_NX/NY`, `VISAO_MEDIA_INTERVALO`, `FUNDO_ATUALIZA`, `FUNDO_ALPHA_SHIFT` |
| Segmentação | `VISAO_SCAN_STEP`, `VISAO_MAX_RUNS`, `CENTROIDE_PONDERADO`, `REFINO_SUBPIXEL` |
| Filtro | `FILTRO_ATIVO`, `FILTRO_ALFA`, `FILTRO_BETA` |
| Tracking | `ROI_FRACAO`, `ROI_PERDE_FRAMES` |
| Câmera | `CAM_AUTO_AJUSTE`, `CAM_AEC_FIXO`, `CAM_AGC_FIXO`, `CAM_CALIBRA_MS`, `CAM_USA_NVS` |
| Geometria | `MESA_LADO_CM`, `MESA_EXT_*`, `MESA_ROI_*`, `MESA_CALIB_*` |

---

## O que mudou (e por quê)

| Antes | Agora | Ganho |
|---|---|---|
| Um único blob global (soma de TODOS os pixels) | Componentes conexos + escolha por score | **Posição correta** com reflexos/manchas |
| Exposição/ganho/AWB em automático | Fixos + calibração na NVS | Brilho estável → limiar confiável |
| Média global da mesa + delta | Grade local 4×4 ou subtração de fundo | Imune a iluminação desigual |
| Centróide binário | Ponderado pela intensidade + refino sub-pixel | Menos jitter |
| Eixos/escala médios para px→cm | Homografia 3×3 pré-computada | Mais preciso nos cantos **e** mais barato |
| Geometria recalculada por frame | Homografia + LUT de linhas pré-computadas | Menos CPU/frame |
| Medição crua | Filtro α-β (posição + velocidade) | Suave, dá `v` p/ o termo D |
| Debug destruía a imagem (Sobel por cima) | Imagem original + overlays; Sobel opcional | Calibração visual real |
