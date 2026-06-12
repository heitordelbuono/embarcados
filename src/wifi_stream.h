#pragma once
#include <stdbool.h>

// Conecta ao WiFi e sobe o servidor HTTP de streaming MJPEG.
// Uso: wifi_stream_inicia() no boot; o resto roda autonomamente.

void wifi_stream_inicia();  // bloqueia ate conectar
