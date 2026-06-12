#pragma once
// Teste da camera SEM WiFi (Etapa 1, MODO_CAMERA = 1):
//  1) manda 1 foto JPEG pela serial (salvar com tools/recebe_foto.py)
//  2) mede o FPS real de captura local (sem transmitir nada). Nao retorna.

void camera_teste_benchmark();
