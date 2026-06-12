#include "driver_atuacao.h"
#include "config.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

static const char* TAG = "DRIVER";

#define PCA9685_MODE1       0x00
#define PCA9685_PRESCALE    0xFE
#define PCA9685_LED0_ON_L   0x06
#define PCA9685_OSC_CLOCK   25000000UL

static i2c_master_bus_handle_t s_bus = NULL;
static i2c_master_dev_handle_t s_dev = NULL;
static bool i2c_ok = false;
static bool s_swap = false;   // troca runtime dos canais X<->Y (tecla 'z')
static float s_neutro_x = SERVO_NEUTRO_X;
static float s_neutro_y = SERVO_NEUTRO_Y;

// canais efetivos (respeitam o swap em runtime)
static int canal_x() { return s_swap ? SERVO_CANAL_Y : SERVO_CANAL_X; }
static int canal_y() { return s_swap ? SERVO_CANAL_X : SERVO_CANAL_Y; }

static float limita_neutro(float v) {
    if (v < SERVO_MIN) return SERVO_MIN;
    if (v > SERVO_MAX) return SERVO_MAX;
    return v;
}

static void nvs_garante_init_driver() {
    static bool feito = false;
    if (feito) return;
    esp_err_t r = nvs_flash_init();
    if (r == ESP_ERR_NVS_NO_FREE_PAGES || r == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }
    feito = true;
}

static void carrega_neutro_nvs() {
    nvs_garante_init_driver();
    nvs_handle_t h;
    if (nvs_open("servo", NVS_READONLY, &h) != ESP_OK) return;
    int32_t nx = 0, ny = 0;
    bool ok = nvs_get_i32(h, "nx10", &nx) == ESP_OK &&
              nvs_get_i32(h, "ny10", &ny) == ESP_OK;
    nvs_close(h);
    if (!ok) return;
    s_neutro_x = limita_neutro(nx / 10.0f);
    s_neutro_y = limita_neutro(ny / 10.0f);
    ESP_LOGI(TAG, "neutro dos servos lido da NVS: X=%.1f Y=%.1f", s_neutro_x, s_neutro_y);
}

static esp_err_t pca_write(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    return i2c_master_transmit(s_dev, buf, 2, 50);
}

static esp_err_t pca_read(uint8_t reg, uint8_t* out) {
    return i2c_master_transmit_receive(s_dev, &reg, 1, out, 1, 10);
}

void DriverAtuacao::iniciaMotores() {
    carrega_neutro_nvs();

    // O PCA9685 NAO cria um barramento proprio: ele se pendura no I2C que a
    // camera (SCCB) ja criou, nos pinos SIOD/SIOC (GPIO 26/27). A camera DEVE
    // ter iniciado antes (ver gerenciador.cpp: visao.inicia() vem antes daqui).
    esp_err_t e = i2c_master_get_bus_handle((i2c_port_num_t)CAM_SCCB_PORT, &s_bus);
    if (e != ESP_OK || s_bus == NULL) {
        ESP_LOGE(TAG, "nao achei o barramento I2C da camera (port %d): %s. A camera iniciou antes?",
                 CAM_SCCB_PORT, esp_err_to_name(e));
        return;
    }

    i2c_device_config_t dev_cfg = {};
    dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_cfg.device_address  = PCA9685_ENDERECO;
    dev_cfg.scl_speed_hz    = I2C_FREQ_HZ;

    e = i2c_master_bus_add_device(s_bus, &dev_cfg, &s_dev);
    if (e != ESP_OK) { ESP_LOGE(TAG, "i2c_master_bus_add_device: %s", esp_err_to_name(e)); return; }

    // A primeira transacao num barramento recem-criado pode falhar (linha ainda
    // subindo com o pull-up interno). Tenta algumas vezes antes de desistir.
    e = ESP_FAIL;
    for (int tentativa = 1; tentativa <= 10; tentativa++) {
        if (i2c_master_probe(s_bus, PCA9685_ENDERECO, 50) == ESP_OK) {
            e = pca_write(PCA9685_MODE1, 0x00);
            if (e == ESP_OK) { ESP_LOGI(TAG, "PCA9685 respondeu na tentativa %d", tentativa); break; }
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "PCA9685 nao respondeu (end=0x%02X) apos 10 tentativas. Confira SDA/SCL e VCC.", PCA9685_ENDERECO);
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(5));

    uint8_t prescale = (uint8_t)((PCA9685_OSC_CLOCK + (PCA9685_FREQ_HZ * 2048UL)) /
                                 (4096UL * PCA9685_FREQ_HZ) - 1);
    uint8_t mode1_old = 0;
    pca_read(PCA9685_MODE1, &mode1_old);
    pca_write(PCA9685_MODE1, (mode1_old & 0x7F) | 0x10);
    pca_write(PCA9685_PRESCALE, prescale);
    pca_write(PCA9685_MODE1, mode1_old & 0x7F);
    vTaskDelay(pdMS_TO_TICKS(5));
    // bit7=RESTART, bit5=AI (auto-increment, obrigatorio para escrita multi-byte)
    pca_write(PCA9685_MODE1, (mode1_old & 0x7F) | 0xA0);

    i2c_ok = true;
    ESP_LOGI(TAG, "PCA9685 OK | prescale=%u (~%d Hz) | SDA=%d SCL=%d",
             prescale, PCA9685_FREQ_HZ, I2C_SDA_GPIO, I2C_SCL_GPIO);

    neutro();
}

void DriverAtuacao::desconecta() {
    // Remove o device PCA do barramento compartilhado da camera. Necessario
    // ANTES de a camera reinicializar (alternaCaptura), senao ela nao consegue
    // deletar/recriar o bus do port 1. Depois chame iniciaMotores() de novo.
    if (s_dev) { i2c_master_bus_rm_device(s_dev); s_dev = NULL; }
    i2c_ok = false;
}

void DriverAtuacao::escreveAngulo(int canal, float angulo) {
    if (angulo > SERVO_MAX) angulo = SERVO_MAX;
    if (angulo < SERVO_MIN) angulo = SERVO_MIN;
    if (!i2c_ok) return;

    float us        = SERVO_PULSO_MIN_US + (angulo / 180.0f) * (SERVO_PULSO_MAX_US - SERVO_PULSO_MIN_US);
    float periodo_us = 1000000.0f / PCA9685_FREQ_HZ;
    uint16_t off    = (uint16_t)(us / periodo_us * 4096.0f + 0.5f);
    if (off > 4095) off = 4095;

    uint8_t reg = PCA9685_LED0_ON_L + 4 * canal;
    uint8_t buf[5] = {reg, 0x00, 0x00, (uint8_t)(off & 0xFF), (uint8_t)(off >> 8)};
    esp_err_t e = i2c_master_transmit(s_dev, buf, 5, 10);
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "falha ao escrever canal %d: %s", canal, esp_err_to_name(e));
        return;
    }
    ESP_LOGD(TAG, "canal %d -> %.1f graus (%.0f us, %u ticks)", canal, angulo, us, off);
}

void DriverAtuacao::escrevePulso(int canal, int us) {
    if (!i2c_ok) { ESP_LOGW(TAG, "I2C nao iniciado"); return; }
    float periodo_us = 1000000.0f / PCA9685_FREQ_HZ;
    uint16_t off = (uint16_t)(us / periodo_us * 4096.0f + 0.5f);
    if (off > 4095) off = 4095;
    uint8_t reg = PCA9685_LED0_ON_L + 4 * canal;
    uint8_t buf[5] = {reg, 0x00, 0x00, (uint8_t)(off & 0xFF), (uint8_t)(off >> 8)};
    i2c_master_transmit(s_dev, buf, 5, 10);
    printf("canal %d -> pulso direto %d us (%u ticks)\n", canal, us, off);
}

void DriverAtuacao::enviaCorrecaoX(float sinal) {
    // sinal em [-SERVO_RANGE, +SERVO_RANGE]
#if SERVO_X_INVERTIDO
    escreveAngulo(canal_x(), s_neutro_x - sinal);
#else
    escreveAngulo(canal_x(), s_neutro_x + sinal);
#endif
}

void DriverAtuacao::enviaCorrecaoY(float sinal) {
#if SERVO_Y_INVERTIDO
    escreveAngulo(canal_y(), s_neutro_y - sinal);
#else
    escreveAngulo(canal_y(), s_neutro_y + sinal);
#endif
}

bool DriverAtuacao::trocaEixos() {
    s_swap = !s_swap;
    return s_swap;
}

void DriverAtuacao::neutro() {
    escreveAngulo(SERVO_CANAL_X, s_neutro_x);
    escreveAngulo(SERVO_CANAL_Y, s_neutro_y);
}

void DriverAtuacao::ajustaNeutroX(float delta) {
    s_neutro_x = limita_neutro(s_neutro_x + delta);
    neutro();
    printf("Neutro servo X = %.1f graus\n", s_neutro_x);
}

void DriverAtuacao::ajustaNeutroY(float delta) {
    s_neutro_y = limita_neutro(s_neutro_y + delta);
    neutro();
    printf("Neutro servo Y = %.1f graus\n", s_neutro_y);
}

void DriverAtuacao::resetNeutro() {
    s_neutro_x = SERVO_NEUTRO_X;
    s_neutro_y = SERVO_NEUTRO_Y;
    neutro();
    printf("Neutro dos servos voltou ao padrao: X=%.1f Y=%.1f\n", s_neutro_x, s_neutro_y);
}

void DriverAtuacao::salvaNeutro() {
    nvs_garante_init_driver();
    nvs_handle_t h;
    if (nvs_open("servo", NVS_READWRITE, &h) != ESP_OK) {
        printf("Falha ao abrir NVS dos servos\n");
        return;
    }
    nvs_set_i32(h, "nx10", (int32_t)lroundf(s_neutro_x * 10.0f));
    nvs_set_i32(h, "ny10", (int32_t)lroundf(s_neutro_y * 10.0f));
    nvs_commit(h);
    nvs_close(h);
    printf("Neutro dos servos salvo: X=%.1f Y=%.1f\n", s_neutro_x, s_neutro_y);
}

float DriverAtuacao::neutroX() const { return s_neutro_x; }
float DriverAtuacao::neutroY() const { return s_neutro_y; }

void DriverAtuacao::debug() {
    printf("\n=== DEBUG PCA9685 ===\n");

    // 1) Scan I2C: testa todos os enderecos 0x08..0x77
    printf("-- I2C scan (barramento I2C_NUM_0, SDA=%d SCL=%d) --\n",
           I2C_SDA_GPIO, I2C_SCL_GPIO);
    bool achou = false;
    for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
        esp_err_t r = i2c_master_probe(s_bus, addr, 50);
        if (r == ESP_OK) {
            printf("  [OK] 0x%02X", addr);
            if (addr == PCA9685_ENDERECO) printf("  <- PCA9685 esperado");
            printf("\n");
            achou = true;
        }
    }
    if (!achou) printf("  Nenhum dispositivo encontrado! Verifique SDA/SCL/VCC.\n");

    if (!i2c_ok) { printf("===================\n\n"); return; }

    // 2) Registradores do PCA9685
    printf("-- Registradores PCA9685 (end=0x%02X) --\n", PCA9685_ENDERECO);
    uint8_t mode1 = 0, prescale = 0;
    pca_read(PCA9685_MODE1,    &mode1);
    pca_read(PCA9685_PRESCALE, &prescale);
    float freq_real = (float)PCA9685_OSC_CLOCK / (4096.0f * (prescale + 1));
    printf("  MODE1    = 0x%02X\n", mode1);
    printf("  PRESCALE = %u  (~%.1f Hz)\n", prescale, freq_real);

    // 3) Canais 0 e 1: le ON_L/H e OFF_L/H e calcula pulso
    for (int ch = 0; ch < 2; ch++) {
        uint8_t regs[4] = {};
        uint8_t base = PCA9685_LED0_ON_L + 4 * ch;
        for (int i = 0; i < 4; i++) pca_read(base + i, &regs[i]);
        uint16_t on_val  = regs[0] | (regs[1] << 8);
        uint16_t off_val = regs[2] | (regs[3] << 8);
        float us = (off_val - on_val) / 4096.0f * (1000000.0f / freq_real);
        printf("  Canal %d: ON=%u OFF=%u -> pulso=%.0f us\n", ch, on_val, off_val, us);
    }
    printf("====================\n\n");
}
