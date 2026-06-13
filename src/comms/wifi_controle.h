#pragma once

#include <stdbool.h>

// Sobe o AP "MesaPID" e o servidor HTTP + WebSocket.
// Chamar APOS g.inicia() (NVS ja inicializado pela visao).
// Acesse http://192.168.4.1 no celular conectado na rede "MesaPID".
//   GET /        -> pagina HTML com canvas animado
//   GET /ws      -> WebSocket persistente (tick/setpoint)
//   GET /health  -> diagnostico (uptime, heap, sockets, WS)
void wifi_controle_inicia();

// Core 1 (loop de visao) publica a posicao/fps atual. Protegido por spinlock.
void wifi_controle_atualiza_posicao(float x, float y, bool achou, float fps);

// Core 1 le o setpoint atual (definido pelo celular via WebSocket).
void wifi_controle_le_setpoint(float* x, float* y);

// Define o setpoint manualmente (ex.: via serial), com clamp interno.
void wifi_controle_setpoint(float x, float y);
