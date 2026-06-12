#pragma once

struct PosicaoWeb {
    float x, y, fps;
    bool  achou;
};

extern volatile PosicaoWeb g_pos_web;
extern volatile float g_setpoint_x;
extern volatile float g_setpoint_y;

// Sobe o AP "MesaPID" e o servidor HTTP.
// Chamar APOS g.inicia() (NVS ja inicializado pela visao).
// Acesse http://192.168.4.1 no celular conectado na rede "MesaPID".
void wifi_controle_inicia();
