// main.c - ESP32-S3 DAQ + Alarm System using ESP-IDF

#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_lcd_1602.h"
#include "i2c-lcd1602.h"
#include "driver/i2c.h"

#define TAG "DAQ"

#define NTC_ADC_CHANNEL     ADC1_CHANNEL_6
#define NTC_ADC_WIDTH       ADC_WIDTH_BIT_12
#define NTC_ATTEN           ADC_ATTEN_DB_11

#define BTN_INC             GPIO_NUM_12
#define BTN_DEC             GPIO_NUM_14
#define BUZZER_PIN          GPIO_NUM_27

#define LED_GREEN           GPIO_NUM_2
#define LED_YELLOW          GPIO_NUM_4
#define LED_BLUE            GPIO_NUM_16
#define LED_WHITE           GPIO_NUM_17

#define I2C_MASTER_SCL_IO   22
#define I2C_MASTER_SDA_IO   21
#define I2C_MASTER_NUM      I2C_NUM_0
#define LCD_ADDR            0x27

static i2c_lcd1602_info_t *lcd_info;
static volatile float alarm_temp = 25.0;
static float ntc_temp = 0.0;
static bool leds_blinking = false;
static bool blink_state = false;
static esp_timer_handle_t blink_timer;

static QueueHandle_t btn_queue;

typedef enum {
    BTN_INC_EVT,
    BTN_DEC_EVT
} button_event_t;

void init_i2c_lcd(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);

    lcd_info = i2c_lcd1602_malloc();
    i2c_lcd1602_init(lcd_info, I2C_MASTER_NUM, LCD_ADDR, 16, 2);
    i2c_lcd1602_backlight_on(lcd_info);
}

float read_ntc_temp() {
    int raw = adc1_get_raw(NTC_ADC_CHANNEL);
    float voltage = (raw / 4095.0) * 3.3;
    float resistance = (3.3 * 10000 / voltage) - 10000;
    float steinhart = resistance / 10000;
    steinhart = log(steinhart);
    steinhart /= 3950;
    steinhart += 1.0 / (25 + 273.15);
    steinhart = 1.0 / steinhart;
    return steinhart - 273.15;
}

void init_pwm_buzzer() {
    ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 1000,
        .duty_resolution = LEDC_TIMER_8_BIT
    };
    ledc_timer_config(&timer);

    ledc_channel_config_t channel = {
        .gpio_num = BUZZER_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0
    };
    ledc_channel_config(&channel);
}

void buzzer_control(bool on) {
    if (on) {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 128);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    } else {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    }
}

void IRAM_ATTR blink_led_timer_callback(void *arg) {
    blink_state = !blink_state;
    gpio_set_level(LED_GREEN, blink_state);
    gpio_set_level(LED_YELLOW, blink_state);
    gpio_set_level(LED_BLUE, blink_state);
    gpio_set_level(LED_WHITE, blink_state);
}

void update_leds(float diff) {
    if (ntc_temp >= alarm_temp) {
        if (!leds_blinking) {
            const esp_timer_create_args_t blink_args = {
                .callback = &blink_led_timer_callback,
                .name = "blink_leds"
            };
            esp_timer_create(&blink_args, &blink_timer);
            esp_timer_start_periodic(blink_timer, 500000);
            leds_blinking = true;
        }
        return;
    }

    if (leds_blinking) {
        esp_timer_stop(blink_timer);
        esp_timer_delete(blink_timer);
        leds_blinking = false;
    }

    gpio_set_level(LED_GREEN, diff <= 20);
    gpio_set_level(LED_YELLOW, diff <= 15);
    gpio_set_level(LED_BLUE, diff <= 10);
    gpio_set_level(LED_WHITE, diff <= 2);
}

static void IRAM_ATTR gpio_isr_handler(void* arg) {
    button_event_t evt = (gpio_num_t)arg == BTN_INC ? BTN_INC_EVT : BTN_DEC_EVT;
    xQueueSendFromISR(btn_queue, &evt, NULL);
}

void setup_buttons() {
    btn_queue = xQueueCreate(10, sizeof(button_event_t));
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BTN_INC) | (1ULL << BTN_DEC),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = 1,
        .intr_type = GPIO_INTR_NEGEDGE
    };
    gpio_config(&io_conf);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(BTN_INC, gpio_isr_handler, (void*)BTN_INC);
    gpio_isr_handler_add(BTN_DEC, gpio_isr_handler, (void*)BTN_DEC);
}

void update_lcd() {
    char line1[17], line2[17];
    snprintf(line1, sizeof(line1), "NTC: %.1f C", ntc_temp);
    snprintf(line2, sizeof(line2), "Alarm: %.1f C", alarm_temp);
    i2c_lcd1602_clear(lcd_info);
    i2c_lcd1602_move_cursor(lcd_info, 0, 0);
    i2c_lcd1602_write_string(lcd_info, line1);
    i2c_lcd1602_move_cursor(lcd_info, 0, 1);
    i2c_lcd1602_write_string(lcd_info, line2);
}

void app_main(void) {
    adc1_config_width(NTC_ADC_WIDTH);
    adc1_config_channel_atten(NTC_ADC_CHANNEL, NTC_ATTEN);

    gpio_set_direction(LED_GREEN, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED_YELLOW, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED_BLUE, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED_WHITE, GPIO_MODE_OUTPUT);

    init_i2c_lcd();
    init_pwm_buzzer();
    setup_buttons();

    button_event_t evt;

    while (1) {
        ntc_temp = read_ntc_temp();

        if (xQueueReceive(btn_queue, &evt, pdMS_TO_TICKS(10))) {
            if (evt == BTN_INC_EVT) alarm_temp += 5;
            if (evt == BTN_DEC_EVT) alarm_temp -= 5;
        }

        update_lcd();
        buzzer_control(ntc_temp >= alarm_temp);
        update_leds(alarm_temp - ntc_temp);

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}
