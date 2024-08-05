#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "led_control.h"
#include "sensors.h"
#include "wifi.h"
#include "http_server.h"

#define TAG "MAIN"

void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    status_led_channel = led_initialize(&status_led_encoder);

    sensors_initialize();

    wifi_init_sta();

    start_http_server();
}
