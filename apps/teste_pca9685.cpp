// ============================================================
//  apps/teste_pca9685.cpp  (era ETAPA 0)
//  TESTE MINIMO DO SERVO via PCA9685 (I2C). Sem camera, sem PID.
//  So fala com o PCA9685 e varre os servos dos canais 0 e 1.
//  Tenta as DUAS orientacoes de SDA/SCL (32/33 e 33/32).
//
//  Para compilar: troque "main.cpp" por "../apps/teste_pca9685.cpp"
//  no SRCS de src/CMakeLists.txt (ver apps/README.md).
// ============================================================
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"

#define T_ADDR  0x40
#define T_FREQ  50            // Hz do servo

static i2c_master_bus_handle_t t_bus = NULL;
static i2c_master_dev_handle_t t_dev = NULL;

static void t_w(uint8_t reg, uint8_t val) {              // escreve 1 registrador
    uint8_t b[2] = {reg, val};
    esp_err_t e = i2c_master_transmit(t_dev, b, 2, 100);
    if (e != ESP_OK) printf("  ERRO escrita reg 0x%02X: %s\n", reg, esp_err_to_name(e));
}

static void t_pulso(int canal, int us) {                 // manda pulso em us
    uint16_t off = (uint16_t)((float)us / (1000000.0f / T_FREQ) * 4096.0f);
    uint8_t reg = 0x06 + 4 * canal;                      // LED0_ON_L + 4*canal
    uint8_t b[5] = {reg, 0, 0, (uint8_t)(off & 0xFF), (uint8_t)(off >> 8)};
    esp_err_t e = i2c_master_transmit(t_dev, b, 5, 100);
    printf("  canal %d -> %d us (off=%u) %s\n", canal, us, off,
           e == ESP_OK ? "OK" : esp_err_to_name(e));
}

// cria o barramento numa orientacao, escaneia, e devolve o 1o endereco que
// deu ACK (ou -1). Limpa o barramento no fim (a menos que ache algo).
static int scan_em(int sda, int scl, bool manter) {
    i2c_master_bus_handle_t bus = NULL;
    i2c_master_bus_config_t bc = {};
    bc.i2c_port          = I2C_NUM_0;
    bc.sda_io_num        = (gpio_num_t)sda;
    bc.scl_io_num        = (gpio_num_t)scl;
    bc.clk_source        = I2C_CLK_SRC_DEFAULT;
    bc.glitch_ignore_cnt = 7;
    bc.flags.enable_internal_pullup = true;
    if (i2c_new_master_bus(&bc, &bus) != ESP_OK) {
        printf("-- nao consegui criar bus SDA=%d SCL=%d --\n", sda, scl);
        return -1;
    }
    printf("-- scan com SDA=%d SCL=%d --\n", sda, scl);
    int achou = -1;
    for (uint8_t a = 0x08; a <= 0x77; a++) {
        if (i2c_master_probe(bus, a, 100) == ESP_OK) {
            printf("  [ACK] 0x%02X%s\n", a, a == T_ADDR ? "  <- PCA9685" : "");
            if (achou < 0) achou = a;
        }
    }
    if (achou < 0) printf("  (nada respondeu nesta orientacao)\n");
    if (achou >= 0 && manter) { t_bus = bus; }   // mantem o bus que funcionou
    else                      { i2c_del_master_bus(bus); }
    return achou;
}

extern "C" void app_main(void) {
    vTaskDelay(pdMS_TO_TICKS(500));
    printf("\n=== TESTE PCA9685: tenta as DUAS orientacoes de SDA/SCL ===\n");

    // Tenta primeiro 32/33; se nada, tenta os fios trocados (33/32).
    int sda_ok = 32, scl_ok = 33;
    int f = scan_em(32, 33, true);
    if (f < 0) { sda_ok = 33; scl_ok = 32; f = scan_em(33, 32, true); }

    if (f < 0) {
        printf("\n>> NINGUEM respondeu em NENHUMA orientacao.\n");
        printf(">> Logo NAO e SDA/SCL trocado nem endereco -> e ENERGIA ou conexao:\n");
        printf(">>   1) VCC do modulo no 3.3V (o LED vermelho do PCA acende?)\n");
        printf(">>   2) GND do PCA ligado no GND do ESP (comum)\n");
        printf(">>   3) os 2 fios bem encaixados em 32 e 33 e nos pinos SDA/SCL do PCA\n");
        return;
    }

    printf("\n>> ACHOU 0x%02X com SDA=%d SCL=%d <<\n", f, sda_ok, scl_ok);
    if (sda_ok != 32) printf(">> ATENCAO: os fios estao TROCADOS! Configure I2C_SDA_GPIO=%d I2C_SCL_GPIO=%d\n", sda_ok, scl_ok);

    // configura o dispositivo no bus que funcionou
    i2c_device_config_t dc = {};
    dc.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dc.device_address  = T_ADDR;
    dc.scl_speed_hz    = 100000;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(t_bus, &dc, &t_dev));

    // configura o PCA9685 pra 50 Hz
    t_w(0x00, 0x00);                                     // MODE1 = normal
    vTaskDelay(pdMS_TO_TICKS(5));
    uint8_t prescale = (uint8_t)(25000000UL / (4096UL * T_FREQ) - 1);  // ~121
    t_w(0x00, 0x10);                                     // sleep p/ mexer no prescale
    t_w(0xFE, prescale);                                 // PRESCALE
    t_w(0x00, 0x00);                                     // acorda
    vTaskDelay(pdMS_TO_TICKS(5));
    t_w(0x00, 0xA0);                                     // restart + auto-increment
    printf("PCA9685 configurado: prescale=%u (~%d Hz). Varrendo canais 0 e 1...\n", prescale, T_FREQ);

    // varre pra sempre: 1.0ms -> 1.5ms -> 2.0ms -> 1.5ms
    for (;;) {
        printf("-- min --\n");    t_pulso(0, 1000); t_pulso(1, 1000); vTaskDelay(pdMS_TO_TICKS(1000));
        printf("-- meio --\n");   t_pulso(0, 1500); t_pulso(1, 1500); vTaskDelay(pdMS_TO_TICKS(1000));
        printf("-- max --\n");    t_pulso(0, 2000); t_pulso(1, 2000); vTaskDelay(pdMS_TO_TICKS(1000));
        printf("-- meio --\n");   t_pulso(0, 1500); t_pulso(1, 1500); vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
