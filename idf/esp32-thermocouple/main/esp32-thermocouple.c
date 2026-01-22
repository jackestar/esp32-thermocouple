#include <stdio.h>
#include <inttypes.h>

#include <hd44780.h>
#include <esp_idf_lib_helpers.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "esp_adc/adc_oneshot.h"
#include "esp_check.h"
#include "esp_log.h"

#include "dht.h"

#define LCD_RW 33
#define LCD_VO 16
#define TER_ADC 2
#define AMB_DHT 4

#define debug 1

static const uint8_t amb_sym[8] = {0b00000100,
                                   0b00000100,
                                   0b00011111,
                                   0b00001110,
                                   0b00000100,
                                   0b00011111,
                                   0b00000100,
                                   0b00000100};
static const uint8_t ter_sym[8] = {0b00011000,
                                   0b00000100,
                                   0b00000010,
                                   0b00000001,
                                   0b00000010,
                                   0b00000100,
                                   0b00011000,
                                   0b00000000};
static const uint8_t cen_sym[8] = {0b00011000,
                                   0b00011000,
                                   0b00000011,
                                   0b00000100,
                                   0b00000100,
                                   0b00000100,
                                   0b00000011,
                                   0b00000000};

hd44780_t lcd = {
    .write_cb = NULL,
    .font = HD44780_FONT_5X8,
    .lines = 2,
    .pins = {
        .rs = GPIO_NUM_21,
        .e = GPIO_NUM_34,
        .d4 = GPIO_NUM_35,
        .d5 = GPIO_NUM_36,
        .d6 = GPIO_NUM_37,
        .d7 = GPIO_NUM_38,
        .bl = HD44780_NOT_USED}};

float amb_tem = 27.00;
float ter_tem = 27.00;
static adc_oneshot_unit_handle_t adc_handle = NULL;

void lcd_config()
{
    gpio_reset_pin(LCD_RW);
    gpio_set_direction(LCD_RW, GPIO_MODE_OUTPUT);
    gpio_set_level(LCD_RW, 0);
    gpio_reset_pin(LCD_VO);
    gpio_set_direction(LCD_VO, GPIO_MODE_OUTPUT);
    gpio_set_level(LCD_VO, 0);

    ESP_ERROR_CHECK(hd44780_init(&lcd));
}

void dht_task(void *pvParameters)
{
    gpio_set_pull_mode(AMB_DHT, GPIO_PULLUP_ONLY);
    while (1)
    {
        // dht_read_float_data(DHT_TYPE_AM2301, AMB_DHT, NULL, &amb_tem);
        ESP_ERROR_CHECK(dht_read_float_data(DHT_TYPE_AM2301, AMB_DHT, NULL, &amb_tem));
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

void lcd_task(void *pvParameters)
{
    hd44780_upload_character(&lcd, 0, amb_sym);
    hd44780_upload_character(&lcd, 1, ter_sym);
    hd44780_upload_character(&lcd, 2, cen_sym);

    hd44780_gotoxy(&lcd, 0, 0);
    hd44780_puts(&lcd, "\x08      \x0A");
    hd44780_gotoxy(&lcd, 0, 1);
    hd44780_puts(&lcd, "\x09      \x0A");

    while (1)
    {
        char buf[16];

        hd44780_gotoxy(&lcd, 2, 1);
        snprintf(buf, sizeof(buf), "%.2f", ter_tem);
        buf[sizeof(buf) - 1] = 0;
        hd44780_puts(&lcd, buf);

        hd44780_gotoxy(&lcd, 2, 0);
        snprintf(buf, sizeof(buf), "%.2f", amb_tem);
        buf[sizeof(buf) - 1] = 0;
        hd44780_puts(&lcd, buf);

        vTaskDelay(pdMS_TO_TICKS(250));
    }
}

#define BLINK_GPIO 15

void led_blink(void *pvParameters)
{
    gpio_reset_pin(BLINK_GPIO);
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);

    while (1)
    {
        // Turn LED ON
        gpio_set_level(BLINK_GPIO, 1);
        vTaskDelay(1000 / portTICK_PERIOD_MS); // Delay 1 second

        // Turn LED OFF
        gpio_set_level(BLINK_GPIO, 0);
        vTaskDelay(1000 / portTICK_PERIOD_MS); // Delay 1 second
    }
}

#define SPS 20

void adc_task(void *pvParameters)
{
    printf("ADC task\n");
    adc_oneshot_chan_cfg_t chan_cfg = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,
    };

    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, TER_ADC, &chan_cfg));
    printf("ADC task: LOOP\n");
    float acumulator[SPS] = {0.0};
    uint8_t counter = 0;
    while (1)
    {
        int raw = 0;
        esp_err_t ret = adc_oneshot_read(adc_handle, TER_ADC, &raw);
        if (ret == ESP_OK)
        {
            const float vref = 3.3f;       /* ADC full-scale for 11dB approx */
            const float max_raw = 4095.0f; /* 12-bit resolution */
            float voltage = ((float)raw) * (vref / max_raw);

            counter++;
            if (counter == SPS)
                counter = 0;

            acumulator[counter] = voltage;
            if (counter == SPS - 1 || counter == SPS / 2)
            {
                float sum = 0;
                for (uint8_t i = 0; i < SPS; i++)
                    sum += acumulator[i];
                sum /= SPS;

                const int bias = 1000; // 0-max_raw
                ter_tem = (max_raw - sum - bias) * 100 / max_raw;

                if (debug)
                {
                    printf("ADC %f\n", sum);
                    printf("AMB %f\n", amb_tem);
                }
            }
        }
        else
        {
            printf("ADC task fail!!!\n");
            ESP_LOGW("adc_task", "adc read failed: %d", ret);
        }

        vTaskDelay(pdMS_TO_TICKS(2000 / SPS));
    }
}

void app_main(void)
{
    lcd_config();
    xTaskCreate(lcd_task, "lcd_task", configMINIMAL_STACK_SIZE * 3, NULL, 5, NULL);
    xTaskCreate(led_blink, "led_blink", configMINIMAL_STACK_SIZE * 3, NULL, 5, NULL);

    adc_oneshot_unit_init_cfg_t init_cfg = {
        .unit_id = ADC_UNIT_2,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_cfg, &adc_handle));

    xTaskCreate(adc_task, "adc_task", configMINIMAL_STACK_SIZE * 3, NULL, 5, NULL);
    xTaskCreate(dht_task, "dht_task", configMINIMAL_STACK_SIZE * 3, NULL, 5, NULL);
}
