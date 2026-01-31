#include <stdio.h>
#include <inttypes.h>
#include <stdlib.h>

#include <hd44780.h>
#include <esp_idf_lib_helpers.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

#include "esp_adc/adc_oneshot.h"
#include "esp_check.h"
#include "esp_log.h"

#include "dht.h"

#include "driver/dac_oneshot.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_http_server.h"
#include "esp_err.h"
#include <string.h>

uint8_t DAC_DIFF = 115; //180
uint8_t DAC_VGND = 215;

float TER_BIAS = 408.4583; // 0-max_raw
int TER_PROP = -568.8889;

// Breadboard test have another easier pin out
#define TEST

#ifdef TEST

#define TER_ADC 8
#define AMB_DHT 5

hd44780_t lcd = {
    .write_cb = NULL,
    .font = HD44780_FONT_5X8,
    .lines = 2,
    .pins = {
        .rs = GPIO_NUM_7,
        .e = GPIO_NUM_3,
        .d4 = GPIO_NUM_8,
        .d5 = GPIO_NUM_6,
        .d6 = GPIO_NUM_4,
        .d7 = GPIO_NUM_2,
        .bl = HD44780_NOT_USED}};

#else

#define TER_ADC 2
#define AMB_DHT 4

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

#define TER_ADC 2
#define AMB_DHT 4

#define LCD_RW 33
#define LCD_VO 16

#endif

#define EXAMPLE_DAC_CHAN0_ADC_CHAN ADC_CHANNEL_6 // GPIO17, same as DAC channel 0
#define EXAMPLE_DAC_CHAN1_ADC_CHAN ADC_CHANNEL_7 // GPIO18, same as DAC channel 1

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

float amb_tem = 27.00;
float ter_tem = 27.00;
float ADC_ACUM = 0;
static adc_oneshot_unit_handle_t adc_handle = NULL;
static dac_oneshot_handle_t dac_chan0_handle = NULL;
static dac_oneshot_handle_t dac_chan1_handle = NULL;

/* Webserver / Wi-Fi globals */
static httpd_handle_t server = NULL;
static SemaphoreHandle_t config_mutex = NULL;

/* Simple root page (no styling) */
static const char *index_html =
    "<!doctype html>"
    "<html><head><meta charset=\"utf-8\"></head><body>"
    "<h3>ESP32 Thermocouple</h3>"
    "<div>Ambient: <span id=\"amb\">--</span> &deg;C</div>"
    "<div>Thermocouple: <span id=\"ter\">--</span> &deg;C</div>"
    "<hr>"
    "<div>DAC_DIFF: <input id=\"DAC_DIFF_in\" size=6> <p id=\"DAC_DIFF_val\">--</p></div>"
    "<div>DAC_VGND: <input id=\"DAC_VGND_in\" size=6> <p id=\"DAC_VGND_val\">--</p></div>"
    "<div>TER_BIAS: <input id=\"TER_BIAS_in\" size=6> <p id=\"TER_BIAS_val\">--</p></div>"
    "<div>TER_PROP: <input id=\"TER_PROP_in\" size=6> <p id=\"TER_PROP_val\">--</p></div>"
    "<div>ADC_ACUM: <input id=\"ADC_ACUM_in\" size=8> <p id=\"ADC_ACUM_val\">--</p></div>"
    "<div><button onclick=\"apply()\">Apply</button></div>"
    "<script>"
    "async function fetchState(){"
    " const r=await fetch('/api/state');"
    " const j=await r.json();"
    " document.getElementById('amb').textContent=j.amb_tem.toFixed(2);"
    " document.getElementById('ter').textContent=j.ter_tem.toFixed(2);"
    " document.getElementById('DAC_DIFF_val').textContent = j.DAC_DIFF;"
    " document.getElementById('DAC_VGND_val').textContent = j.DAC_VGND;"
    " document.getElementById('TER_BIAS_val').textContent = j.TER_BIAS.toFixed(2);"
    " document.getElementById('TER_PROP_val').textContent = j.TER_PROP.toFixed(2);"
    " document.getElementById('ADC_ACUM_val').textContent = j.ADC_ACUM.toFixed(3);"
    "}"
    "async function apply(){"
    " const params=[];"
    " const mappings=[['DAC_DIFF_in','DAC_DIFF'],['DAC_VGND_in','DAC_VGND'],['TER_BIAS_in','TER_BIAS'],['TER_PROP_in','TER_PROP'],['ADC_ACUM_in','ADC_ACUM']];"
    " mappings.forEach(pair=>{ const v=document.getElementById(pair[0]).value; if(v!=='') params.push(encodeURIComponent(pair[1])+'='+encodeURIComponent(v)); });"
    " await fetch('/api/set?'+params.join('&'));"
    " setTimeout(fetchState,200);"
    "}"
    "setInterval(fetchState,1000); fetchState();"
    "</script>"
    "</body></html>";

/* Handler: root page */
static esp_err_t root_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, index_html, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

/* Handler: /api/state -> JSON with current values */
static esp_err_t api_state_get_handler(httpd_req_t *req)
{
    char buf[256];
    float dac_diff, dac_vgnd, ter_bias, ter_prop, adc_acum, amb, ter;

    if (config_mutex) xSemaphoreTake(config_mutex, portMAX_DELAY);
    dac_diff = DAC_DIFF;
    dac_vgnd = DAC_VGND;
    ter_bias = TER_BIAS;
    ter_prop = TER_PROP;
    adc_acum = ADC_ACUM;
    amb = amb_tem;
    ter = ter_tem;
    if (config_mutex) xSemaphoreGive(config_mutex);

    int len = snprintf(buf, sizeof(buf), "{\"DAC_DIFF\":%.0f,\"DAC_VGND\":%.0f,\"TER_BIAS\":%.2f,\"TER_PROP\":%.2f,\"ADC_ACUM\":%.3f,\"amb_tem\":%.2f,\"ter_tem\":%.2f}", dac_diff, dac_vgnd, ter_bias, ter_prop, adc_acum, amb, ter);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, buf, len);
    return ESP_OK;
}

/* Handler: /api/set?KEY=VALUE&... -> update variables */
static esp_err_t api_set_get_handler(httpd_req_t *req)
{
    char buf[256];
    if (httpd_req_get_url_query_str(req, buf, sizeof(buf)) == ESP_OK)
    {
        char val[64];
        if (config_mutex) xSemaphoreTake(config_mutex, portMAX_DELAY);

        if (httpd_query_key_value(buf, "DAC_DIFF", val, sizeof(val)) == ESP_OK)
            DAC_DIFF = (uint8_t)atoi(val);
        if (httpd_query_key_value(buf, "DAC_VGND", val, sizeof(val)) == ESP_OK)
            DAC_VGND = (uint8_t)atoi(val);
        if (httpd_query_key_value(buf, "TER_BIAS", val, sizeof(val)) == ESP_OK)
            TER_BIAS = atof(val);
        if (httpd_query_key_value(buf, "TER_PROP", val, sizeof(val)) == ESP_OK)
            TER_PROP = atoi(val);
        if (httpd_query_key_value(buf, "ADC_ACUM", val, sizeof(val)) == ESP_OK)
            ADC_ACUM = atof(val);

        if (config_mutex) xSemaphoreGive(config_mutex);
    }

    httpd_resp_sendstr(req, "OK");
    return ESP_OK;
}

/* Start the webserver and register URIs */
static httpd_handle_t start_webserver(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t server = NULL;
    if (httpd_start(&server, &config) == ESP_OK)
    {
        httpd_uri_t uri_root = {
            .uri = "/",
            .method = HTTP_GET,
            .handler = root_get_handler,
            .user_ctx = NULL};
        httpd_register_uri_handler(server, &uri_root);

        httpd_uri_t uri_state = {
            .uri = "/api/state",
            .method = HTTP_GET,
            .handler = api_state_get_handler,
            .user_ctx = NULL};
        httpd_register_uri_handler(server, &uri_state);

        httpd_uri_t uri_set = {
            .uri = "/api/set",
            .method = HTTP_GET,
            .handler = api_set_get_handler,
            .user_ctx = NULL};
        httpd_register_uri_handler(server, &uri_set);
    }
    return server;
}

/* Wi-Fi soft AP initializer (open AP) */
static void wifi_init_softap(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    wifi_config_t wifi_config = {0};
    snprintf((char *)wifi_config.ap.ssid, sizeof(wifi_config.ap.ssid), "%s", "ESP32_THERMO");
    wifi_config.ap.ssid_len = strlen("ESP32_THERMO");
    wifi_config.ap.channel = 1;
    wifi_config.ap.max_connection = 4;
    wifi_config.ap.authmode = WIFI_AUTH_OPEN;

    esp_wifi_set_mode(WIFI_MODE_AP);
    esp_wifi_set_config(WIFI_IF_AP, &wifi_config);
    esp_wifi_start();
}

void lcd_config()
{
    ESP_ERROR_CHECK(hd44780_init(&lcd));
}

void dht_task(void *pvParameters)
{
    gpio_set_pull_mode(AMB_DHT, GPIO_PULLUP_ONLY);
    while (1)
    {
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

#define SPS 100

void adc_task(void *pvParameters)
{
    printf("ADC task\n");
    adc_oneshot_chan_cfg_t chan_cfg = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,
    };

    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, TER_ADC, &chan_cfg));
    printf("ADC task: LOOP\n");
    int acumulator[SPS] = {0};
    uint8_t counter = 0;
    while (1)
    {
        int raw = 0;
        esp_err_t ret = adc_oneshot_read(adc_handle, TER_ADC, &raw);
        if (ret == ESP_OK)
        {
            const int max_raw = 8192; /* 13-bit resolution */

            counter++;
            if (counter == SPS)
                counter = 0;

            acumulator[counter] = raw;
            if (counter == SPS - 1)
            {
                ADC_ACUM = 0;
                for (uint8_t i = 0; i < SPS; i++)
                    ADC_ACUM += acumulator[i];
                ADC_ACUM /= SPS;

                const float cast = (max_raw - ADC_ACUM) * TER_PROP;
                ter_tem = amb_tem + TER_BIAS + cast / max_raw;

                if (debug)
                {
                    printf("ADC %f\n", ADC_ACUM);
                    printf("AMB %f\n", amb_tem);
                }
            }
        }
        else
        {
            printf("ADC task fail!!!\n");
            ESP_LOGW("adc_task", "adc read failed: %d", ret);
        }

        vTaskDelay(pdMS_TO_TICKS(1000 / SPS));
    }
}

void dac_update_task(void *pvParameters)
{
    while (1)
    {
        uint8_t diff = 0, vgnd = 0;
        if (config_mutex) xSemaphoreTake(config_mutex, portMAX_DELAY);
        diff = DAC_DIFF;
        vgnd = DAC_VGND;
        if (config_mutex) xSemaphoreGive(config_mutex);

        if (dac_chan0_handle)
            ESP_ERROR_CHECK(dac_oneshot_output_voltage(dac_chan0_handle, diff));
        if (dac_chan1_handle)
            ESP_ERROR_CHECK(dac_oneshot_output_voltage(dac_chan1_handle, vgnd));

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

void app_main(void)
{
    /* DAC oneshot init */
    dac_oneshot_config_t chan0_cfg = {
        .chan_id = DAC_CHAN_0,
    };
    ESP_ERROR_CHECK(dac_oneshot_new_channel(&chan0_cfg, &dac_chan0_handle));

    dac_oneshot_config_t chan1_cfg = {
        .chan_id = DAC_CHAN_1,
    };
    ESP_ERROR_CHECK(dac_oneshot_new_channel(&chan1_cfg, &dac_chan1_handle));

    lcd_config();
    xTaskCreate(lcd_task, "lcd_task", configMINIMAL_STACK_SIZE * 3, NULL, 5, NULL);
    xTaskCreate(led_blink, "led_blink", configMINIMAL_STACK_SIZE * 3, NULL, 5, NULL);

    adc_oneshot_unit_init_cfg_t init_cfg = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_cfg, &adc_handle));

    xTaskCreate(adc_task, "adc_task", configMINIMAL_STACK_SIZE * 3, NULL, 5, NULL);
    xTaskCreate(dht_task, "dht_task", configMINIMAL_STACK_SIZE * 3, NULL, 5, NULL);

    /* Create mutex for protecting config/state shared with web handlers */
    config_mutex = xSemaphoreCreateMutex();

    /* Start Wi-Fi soft AP and webserver */
    wifi_init_softap();
    server = start_webserver();

    /* Start DAC updater task */
    xTaskCreate(dac_update_task, "dac_update", configMINIMAL_STACK_SIZE * 2, NULL, 5, NULL);
}