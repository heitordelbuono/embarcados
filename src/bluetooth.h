#pragma once
// Modulo de comunicacao (BLE): botao liga/desliga, setpoint, ganhos, telemetria.
// No projeto original a posicao da bola vinha por aqui; agora ela vem da visao,
// entao este modulo fica so com comandos do usuario e telemetria.

class Bluetooth {
public:
    void iniciaConexao();      // inicia BLE "Mesa PID" (proximos passos)
    bool temComando();         // ha comando novo do usuario?
    // TODO (proximos passos): getSetpointX/Y, botaoLigaDesliga, botaoCalibra, enviaTelemetria
};
