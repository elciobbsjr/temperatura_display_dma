#include <stdio.h>
#include <string.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/dma.h"
#include "hardware/i2c.h"
#include "inc/ssd1306.h"

#define I2C_PORT i2c1
#define SDA_PIN 14
#define SCL_PIN 15

#define OLED_WIDTH 128
#define OLED_HEIGHT 64

// Definindo M_PI caso não esteja disponível
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

uint16_t adc_buffer[1];

float adc_to_temp(uint16_t raw_adc) {
    const float V_ref = 3.3f;
    const float adc_max = 4095.0f;
    float voltage = raw_adc * V_ref / adc_max;
    return 27.0f - (voltage - 0.706f) / 0.001721f;
}

void init_i2c() {
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);
}

void draw_sun(uint8_t *buffer, int center_x, int center_y, int frame) {
    const int radius = 10;
    const int num_rays = 8;
    const float angle_step = 2 * M_PI / num_rays;

    for (int angle_deg = 0; angle_deg < 360; angle_deg += 10) {
        float angle = angle_deg * M_PI / 180.0f;
        int x = center_x + (int)(radius * cosf(angle));
        int y = center_y + (int)(radius * sinf(angle));
        ssd1306_set_pixel(buffer, x, y, true);
    }

    for (int i = 0; i < num_rays; i++) {
        float angle = angle_step * i + frame * 0.2f;
        int x_end = center_x + (int)((radius + 5) * cosf(angle));
        int y_end = center_y + (int)((radius + 5) * sinf(angle));
        ssd1306_draw_line(buffer, center_x, center_y, x_end, y_end, true);
    }
}

int main() {
    stdio_init_all();

    init_i2c();

    ssd1306_t oled;
    ssd1306_init_bm(&oled, OLED_WIDTH, OLED_HEIGHT, false, ssd1306_i2c_address, I2C_PORT);
    ssd1306_config(&oled);
    ssd1306_init();  // Garante inicialização completa via comandos diretos

    adc_init();
    adc_set_temp_sensor_enabled(true);
    adc_select_input(4);
    adc_fifo_setup(true, true, 1, false, false);

    int dma_chan = dma_claim_unused_channel(true);
    dma_channel_config dma_cfg = dma_channel_get_default_config(dma_chan);
    channel_config_set_transfer_data_size(&dma_cfg, DMA_SIZE_16);
    channel_config_set_read_increment(&dma_cfg, false);
    channel_config_set_write_increment(&dma_cfg, false);
    dma_channel_configure(dma_chan, &dma_cfg, adc_buffer, &adc_hw->fifo, 1, false);

    char display_buffer[32];
    int frame = 0;
    uint32_t last_temp_time = to_ms_since_boot(get_absolute_time());
    float temp_c = 0.0f;

    while (true) {
        uint32_t now = to_ms_since_boot(get_absolute_time());
        if (now - last_temp_time >= 1000) {
            adc_run(true);
            dma_channel_start(dma_chan);
            dma_channel_wait_for_finish_blocking(dma_chan);
            adc_run(false);

            temp_c = adc_to_temp(adc_buffer[0]);
            last_temp_time = now;
        }

        snprintf(display_buffer, sizeof(display_buffer), "Temp: %d°C", (int)temp_c);

        printf("Temperatura Interna: %.2f °C\n", temp_c);

        memset(oled.ram_buffer + 1, 0, oled.bufsize - 1);

        draw_sun(oled.ram_buffer + 1, OLED_WIDTH / 2, OLED_HEIGHT / 3, frame);

        ssd1306_draw_string(oled.ram_buffer + 1, 24, 50, display_buffer);

        ssd1306_command(&oled, ssd1306_set_column_address);
        ssd1306_command(&oled, 0);
        ssd1306_command(&oled, OLED_WIDTH - 1);

        ssd1306_command(&oled, ssd1306_set_page_address);
        ssd1306_command(&oled, 0);
        ssd1306_command(&oled, (OLED_HEIGHT / 8) - 1);

        ssd1306_send_data(&oled);

        frame++;
        sleep_ms(100);  // Suaviza a animação
    }

    return 0;
}
