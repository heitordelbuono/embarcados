#include "gerenciador.h"
#include "maquina_estados.h"
#include "esp_timer.h"
#include "esp_log.h"

static const char* TAG = "GEREN";

void ModuloGerenciador::inicia() {
    controladorX.inicia();
    controladorY.inicia();
    driver.iniciaMotores();
    driver.neutro();
    bluetooth.iniciaConexao();
    visao.inicia();
    estadoAtual = ESPERANDO;
    ESP_LOGI(TAG, "gerenciador iniciado");
}

void ModuloGerenciador::calculaAcaoControle(const Medicao& m) {
    if (!m.achou) return;
    float sinalX = controladorX.correcao(setpointX, m.x);
    float sinalY = controladorY.correcao(setpointY, m.y);
    int64_t agora = esp_timer_get_time();
    fila.push(agora + ATUACAO_ATRASO_US, CORRECAO_X, sinalX);
    fila.push(agora + ATUACAO_ATRASO_US, CORRECAO_Y, sinalY);
}

void ModuloGerenciador::processaFila() {
    int64_t agora = esp_timer_get_time();
    while (!fila.vazia() && fila.evento[0].instante <= agora) {
        DadosEvento e = fila.lerProximoEvento();
        if      (e.tipo == CORRECAO_X) driver.enviaCorrecaoX(e.dado);
        else if (e.tipo == CORRECAO_Y) driver.enviaCorrecaoY(e.dado);
    }
}

void ModuloGerenciador::trataEvento(int evento) {
    Transicao t = matrizTransicao[estadoAtual][evento];
    // TODO: executar a acao t.acao (ligar/desligar driver, etc.)
    estadoAtual = t.proxEstado;
}
