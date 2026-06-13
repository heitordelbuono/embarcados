#include "gerenciador.h"
#include "esp_timer.h"
#include "esp_log.h"

static const char* TAG = "GEREN";

void ModuloGerenciador::inicia() {
    controladorX.inicia();
    controladorY.inicia();
    // A camera (esp_camera_init) reconfigura o IO MUX e atropela o barramento
    // I2C se ele for criado antes. Por isso a camera vem PRIMEIRO e o servo
    // (driver_atuacao) configura o I2C por ULTIMO, ficando dono dos pinos.
    visao.inicia();
    driver.iniciaMotores();
    driver.neutro();
    ESP_LOGI(TAG, "gerenciador iniciado");
}

void ModuloGerenciador::calculaAcaoControle(const Medicao& m, bool ativaX, bool ativaY) {
    if (!m.achou) return;
    // Usa o dt real entre frames (medido pelo filtro alfa-beta) para que a
    // derivada e o integral do PID sejam corretos independente do FPS atual.
    if (m.dt > 0.005f && m.dt < 0.5f) {
        controladorX.dt = m.dt;
        controladorX.derivadaFiltrada.dt = m.dt;
        controladorY.dt = m.dt;
        controladorY.derivadaFiltrada.dt = m.dt;
    }
    // Erro = setpoint - posicao. setpointX/Y sao (0,0) = centro da mesa por padrao.
    float sinalX = ativaX ? controladorX.correcao(setpointX, m.x) : 0.0f;
    float sinalY = ativaY ? controladorY.correcao(setpointY, m.y) : 0.0f;
    // escala global (ganhos efetivos 10x menores) antes de saturar
    sinalX *= PID_ESCALA_SAIDA;
    sinalY *= PID_ESCALA_SAIDA;
    // satura a saida do PID em +- SERVO_RANGE graus a partir do neutro
    if (sinalX >  SERVO_RANGE) sinalX =  SERVO_RANGE;
    if (sinalX < -SERVO_RANGE) sinalX = -SERVO_RANGE;
    if (sinalY >  SERVO_RANGE) sinalY =  SERVO_RANGE;
    if (sinalY < -SERVO_RANGE) sinalY = -SERVO_RANGE;
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
