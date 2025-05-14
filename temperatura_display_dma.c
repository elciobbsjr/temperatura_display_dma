#include <stdio.h>
#include <string.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/dma.h"
#include "hardware/i2c.h"
#include "hardware/rtc.h"
#include "inc/ssd1306.h"

// Definições de configuração do barramento I2C e display OLED
#define I2C_PORT i2c1
#define SDA_PIN 14
#define SCL_PIN 15

#define OLED_WIDTH 128
#define OLED_HEIGHT 64
#define SAMPLE_COUNT 100  // Número de amostras por ciclo de leitura de temperatura

#ifndef M_PI
#define M_PI 3.14159265358979323846  // Definindo PI
#endif

// Buffer para armazenar as leituras do ADC e flag de controle de DMA
uint16_t adc_buffer[SAMPLE_COUNT];
volatile bool dma_transfer_done = false;  // Flag para indicar término da transferência via DMA

// Função para converter a leitura bruta do ADC para temperatura em Celsius
float adc_to_temp(uint16_t raw_adc) {
    const float V_ref = 3.3f;      // Tensão de referência do ADC
    const float adc_max = 4095.0f; // Valor máximo de 12 bits (2^12 - 1)
    float voltage = raw_adc * V_ref / adc_max;
    // Fórmula oficial do datasheet do RP2040 para converter em temperatura
    return 27.0f - (voltage - 0.706f) / 0.001721f;
}

// Handler da interrupção do DMA para indicar que a transferência terminou
void dma_handler() {
    dma_hw->ints0 = 1u << 0;  // Limpa o sinal de interrupção do canal 0
    dma_transfer_done = true; // Sinaliza que a transferência foi concluída
}

// Inicializa a comunicação I2C com o display OLED
void init_i2c() {
    i2c_init(I2C_PORT, 400 * 1000);  // Configura I2C em 400kHz
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);  // Habilita pull-up para garantir integridade no barramento
    gpio_pull_up(SCL_PIN);
}

// Função para desenhar uma animação de "Sol" girando na tela OLED
void draw_sun(uint8_t *buffer, int center_x, int center_y, int frame) {
    const int radius = 10;          // Raio do sol
    const int num_rays = 8;          // Número de raios
    const float angle_step = 2 * M_PI / num_rays;  // Passo angular para os raios

    // Desenho do círculo central do sol
    for (int angle_deg = 0; angle_deg < 360; angle_deg += 10) {
        float angle = angle_deg * M_PI / 180.0f;
        int x = center_x + (int)(radius * cosf(angle));
        int y = center_y + (int)(radius * sinf(angle));
        ssd1306_set_pixel(buffer, x, y, true);
    }

    // Desenho dos raios do sol (animação giratória)
    for (int i = 0; i < num_rays; i++) {
        float angle = angle_step * i + frame * 0.2f;  // Inclui o frame para dar rotação
        int x_end = center_x + (int)((radius + 5) * cosf(angle));
        int y_end = center_y + (int)((radius + 5) * sinf(angle));
        ssd1306_draw_line(buffer, center_x, center_y, x_end, y_end, true);
    }
}

// Função responsável por realizar a leitura da temperatura usando DMA
float read_temperature_average_dma(int dma_chan) {
    dma_transfer_done = false;  // Reseta a flag de controle

    // Configura o canal DMA para transferir dados de 16 bits (uint16_t)
    dma_channel_config dma_cfg = dma_channel_get_default_config(dma_chan);
    channel_config_set_transfer_data_size(&dma_cfg, DMA_SIZE_16);
    channel_config_set_read_increment(&dma_cfg, false);
    channel_config_set_write_increment(&dma_cfg, true);  // Incrementa para armazenar no buffer

    // Configura o DMA para transferir os dados do ADC para o buffer na RAM
    dma_channel_configure(
        dma_chan, &dma_cfg, 
        adc_buffer, 
        &adc_hw->fifo, 
        SAMPLE_COUNT, 
        false
    );

    adc_fifo_drain();      // Garante que o FIFO do ADC está vazio
    adc_run(true);         // Inicia as conversões do ADC
    dma_channel_start(dma_chan);  // Inicia a transferência via DMA

    // Aguarda a conclusão da transferência de forma eficiente
    while (!dma_transfer_done) {
        tight_loop_contents();
    }

    adc_run(false);  // Encerra as conversões do ADC

    // Calcula a média da temperatura com base nas amostras
    float temp_sum = 0.0f;
    for (int i = 0; i < SAMPLE_COUNT; i++) {
        temp_sum += adc_to_temp(adc_buffer[i]);
    }
    return temp_sum / SAMPLE_COUNT;
}

int main() {
    stdio_init_all();  // Inicializa a saída padrão (para usar printf no terminal)

    init_i2c();        // Inicializa a comunicação I2C com o OLED

    // Configura e inicializa o display OLED
    ssd1306_t oled;
    ssd1306_init_bm(&oled, OLED_WIDTH, OLED_HEIGHT, false, ssd1306_i2c_address, I2C_PORT);
    ssd1306_config(&oled);
    ssd1306_init();

    // Configura o ADC para o sensor de temperatura interno (canal 4)
    adc_init();
    adc_set_temp_sensor_enabled(true);
    adc_select_input(4);
    adc_fifo_setup(true, true, 1, false, false);  // Configura o FIFO do ADC

    // Configura o canal DMA e a interrupção associada
    int dma_chan = dma_claim_unused_channel(true);
    dma_channel_set_irq0_enabled(dma_chan, true);
    irq_set_exclusive_handler(DMA_IRQ_0, dma_handler);
    irq_set_enabled(DMA_IRQ_0, true);

    char display_buffer[32];  // Buffer para exibir a temperatura formatada
    int frame = 0;            // Contador de frames para a animação
    absolute_time_t last_update_time = get_absolute_time();  // Controle de tempo
    float temp_c = 0.0f;      // Armazena a última leitura de temperatura

    // Loop principal do programa
    while (true) {
        // Verifica se já se passaram 500ms para atualizar a leitura de temperatura
        if (absolute_time_diff_us(last_update_time, get_absolute_time()) >= 500000) {
            temp_c = read_temperature_average_dma(dma_chan);  // Lê temperatura via DMA
            snprintf(display_buffer, sizeof(display_buffer), "Temp: %.1f°C", temp_c);  // Formata a string
            printf("Temperatura Média: %.2f °C\n", temp_c);  // Exibe no terminal (debug)
            last_update_time = get_absolute_time();  // Atualiza o tempo da última medição
        }

        // Limpa o buffer da tela para desenhar novos gráficos e textos
        memset(oled.ram_buffer + 1, 0, oled.bufsize - 1);
        draw_sun(oled.ram_buffer + 1, OLED_WIDTH / 2, OLED_HEIGHT / 3, frame);  // Desenha o "Sol" animado
        ssd1306_draw_string(oled.ram_buffer + 1, 24, 50, display_buffer);       // Escreve a temperatura

        // Envia comandos para atualizar o display OLED
        ssd1306_command(&oled, ssd1306_set_column_address);
        ssd1306_command(&oled, 0);
        ssd1306_command(&oled, OLED_WIDTH - 1);

        ssd1306_command(&oled, ssd1306_set_page_address);
        ssd1306_command(&oled, 0);
        ssd1306_command(&oled, (OLED_HEIGHT / 8) - 1);

        ssd1306_send_data(&oled);  // Atualiza o display com o conteúdo do buffer

        frame++;  // Incrementa o frame da animação do Sol
        sleep_ms(50);  // Delay curto para suavizar a animação sem travar o sistema
    }

    return 0;
}
