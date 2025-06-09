#include <stdio.h>
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

// Endereço do PCF8574 (depende do módulo)
#define LCD_ADDR        0x27
#define I2C_MASTER_SCL  7
#define I2C_MASTER_SDA  6
#define I2C_MASTER_PORT I2C_NUM_0
#define I2C_FREQ_HZ     100000

#define ACK_CHECK_EN    0x1
#define WRITE_BIT       I2C_MASTER_WRITE

static const char *TAG = "LCD";

// Protótipos
void lcd_init();
void lcd_send_cmd(char cmd);
void lcd_send_data(char data);
void lcd_send_string(const char* str);
void lcd_write_nibble(uint8_t nibble, uint8_t rs);
void lcd_send_byte(uint8_t data, uint8_t rs);
void i2c_master_init();

// Função principal
void app_main(void) {
    i2c_master_init();
    lcd_init();
    
    lcd_send_string("Boas-vindas!");
    vTaskDelay(pdMS_TO_TICKS(2000));
    lcd_send_cmd(0xC0); // Linha 2
    lcd_send_string("ESP32-C6 + LCD");
}

// Inicializa o driver I2C
void i2c_master_init() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA,
        .scl_io_num = I2C_MASTER_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_PORT, &conf);
    i2c_driver_install(I2C_MASTER_PORT, conf.mode, 0, 0, 0);
}

// Envia comando para o LCD
void lcd_send_cmd(char cmd) {
    lcd_send_byte(cmd, 0);
}

// Envia dado para o LCD
void lcd_send_data(char data) {
    lcd_send_byte(data, 1);
}

// Escreve uma string
void lcd_send_string(const char* str) {
    while (*str) {
        lcd_send_data(*str++);
    }
}

// Envia byte dividido em dois nibbles
void lcd_send_byte(uint8_t data, uint8_t rs) {
    lcd_write_nibble(data >> 4, rs);
    lcd_write_nibble(data & 0x0F, rs);
    vTaskDelay(pdMS_TO_TICKS(2));
}

// Escreve nibble no barramento
void lcd_write_nibble(uint8_t nibble, uint8_t rs) {
    uint8_t data = (nibble << 4) | (rs ? 0x01 : 0x00) | 0x08; // Backlight ON
    uint8_t en_pulse[4] = {
        data | 0x04, // EN=1
        data,        // EN=0
    };

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (LCD_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
    for (int i = 0; i < 2; i++) {
        i2c_master_write_byte(cmd, en_pulse[i], ACK_CHECK_EN);
    }
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_PORT, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
}

// Inicializa o LCD em 4-bit
void lcd_init() {
    vTaskDelay(pdMS_TO_TICKS(50)); // Aguarda LCD iniciar
    lcd_write_nibble(0x03, 0); // Modo 8-bit
    vTaskDelay(pdMS_TO_TICKS(5));
    lcd_write_nibble(0x03, 0);
    vTaskDelay(pdMS_TO_TICKS(5));
    lcd_write_nibble(0x03, 0);
    vTaskDelay(pdMS_TO_TICKS(5));
    lcd_write_nibble(0x02, 0); // Modo 4-bit

    lcd_send_cmd(0x28); // 2 linhas, 5x8 pontos
    lcd_send_cmd(0x0C); // Display ON, cursor OFF
    lcd_send_cmd(0x06); // Incrementa cursor
    lcd_send_cmd(0x01); // Limpa display
    vTaskDelay(pdMS_TO_TICKS(5));
}
