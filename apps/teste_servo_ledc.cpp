// ============================================================
//  apps/teste_servo_ledc.cpp  (era ETAPA 9)
//  SERVO DIRETO por PWM (LEDC), SEM PCA9685, SEM I2C.
//  Liga o fio de SINAL do servo direto no GPIO. Sem pull-up, sem nada.
//  Servo 1 -> GPIO 32 | Servo 2 -> GPIO 33 | 5V e GND do servo a parte,
//  com GND COMUM no ESP. Se mexer aqui, o problema era o PCA9685/I2C.
//
//  Para compilar: troque "main.cpp" por "../apps/teste_servo_ledc.cpp"
//  no SRCS de src/CMakeLists.txt (ver apps/README.md).
// ============================================================
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"

#define S1_GPIO     32
#define S2_GPIO     33
#define S_FREQ      50                 // Hz do servo
#define S_RES       LEDC_TIMER_16_BIT  // 0..65535 numa janela de 20 ms

// converte largura de pulso (us) em duty de 16 bits (periodo = 20000 us)
static uint32_t us2duty(int us) {
    return (uint32_t)((float)us / 20000.0f * 65536.0f);
}

static void servo_us(ledc_channel_t ch, int us) {
    ledc_set_duty(LEDC_LOW_SPEED_MODE, ch, us2duty(us));
    ledc_update_duty(LEDC_LOW_SPEED_MODE, ch);
    printf("  ch%d -> %d us (duty=%lu)\n", (int)ch, us, (unsigned long)us2duty(us));
}

extern "C" void app_main(void) {
    vTaskDelay(pdMS_TO_TICKS(500));
    printf("\n=== SERVO DIRETO (LEDC, SEM PCA9685) S1=GPIO%d S2=GPIO%d ===\n", S1_GPIO, S2_GPIO);

    // 1) timer LEDC a 50 Hz
    ledc_timer_config_t tcfg = {};
    tcfg.speed_mode      = LEDC_LOW_SPEED_MODE;
    tcfg.timer_num       = LEDC_TIMER_0;
    tcfg.duty_resolution = S_RES;
    tcfg.freq_hz         = S_FREQ;
    tcfg.clk_cfg         = LEDC_AUTO_CLK;
    ESP_ERROR_CHECK(ledc_timer_config(&tcfg));

    // 2) dois canais, um por GPIO
    ledc_channel_config_t ccfg = {};
    ccfg.speed_mode = LEDC_LOW_SPEED_MODE;
    ccfg.timer_sel  = LEDC_TIMER_0;
    ccfg.intr_type  = LEDC_INTR_DISABLE;
    ccfg.hpoint     = 0;
    ccfg.duty       = 0;

    ccfg.gpio_num = S1_GPIO; ccfg.channel = LEDC_CHANNEL_0; ESP_ERROR_CHECK(ledc_channel_config(&ccfg));
    ccfg.gpio_num = S2_GPIO; ccfg.channel = LEDC_CHANNEL_1; ESP_ERROR_CHECK(ledc_channel_config(&ccfg));

    printf("LEDC pronto. Varrendo os dois servos...\n");

    // 3) varre pra sempre: 1.0 -> 1.5 -> 2.0 -> 1.5 ms
    for (;;) {
        printf("-- min (1.0ms) --\n");  servo_us(LEDC_CHANNEL_0, 1000); servo_us(LEDC_CHANNEL_1, 1000); vTaskDelay(pdMS_TO_TICKS(1000));
        printf("-- meio (1.5ms) --\n"); servo_us(LEDC_CHANNEL_0, 1500); servo_us(LEDC_CHANNEL_1, 1500); vTaskDelay(pdMS_TO_TICKS(1000));
        printf("-- max (2.0ms) --\n");  servo_us(LEDC_CHANNEL_0, 2000); servo_us(LEDC_CHANNEL_1, 2000); vTaskDelay(pdMS_TO_TICKS(1000));
        printf("-- meio (1.5ms) --\n"); servo_us(LEDC_CHANNEL_0, 1500); servo_us(LEDC_CHANNEL_1, 1500); vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
