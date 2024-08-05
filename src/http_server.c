#include <esp_http_server.h>
#include <esp_log.h>
#include "sensors.h"

#define TAG "HTTP_SERVER"

// HTTP GET handler for sensor data
esp_err_t sensor_data_get_handler(httpd_req_t *request) {
    char response[512];

    esp_err_t ens160_result, aht21_result;
    struct ens160_data ens160_data = {0};
    struct aht21_data aht21_data = {0};
    
    // Read data from the sensors
    ens160_result = ens160_read_data(&ens160_data);
    aht21_result = aht21_read_data(&aht21_data);
    
    size_t offset = 0;
    offset += snprintf(response + offset, sizeof(response) - offset, "{");
    
    if (ens160_result == ESP_OK) {
        offset += snprintf(response + offset, sizeof(response) - offset, 
                           "\"ens160\": {\"aqi\": %d, \"tvoc_ppb\": %d, \"eco2_ppm\": %d, \"temperature_c\": %.2f, \"humidity_percent\": %.2f}", 
                           ens160_data.aqi, ens160_data.tvoc_ppb, ens160_data.eco2_ppm, ens160_data.temperature_c, ens160_data.humidity_percent);
    } else {
        offset += snprintf(response + offset, sizeof(response) - offset, "\"ens160\": {\"error\": %d}", ens160_result);
    }
    
    if (aht21_result == ESP_OK) {
        offset += snprintf(response + offset, sizeof(response) - offset, 
                           ", \"aht21\": {\"temperature_c\": %.2f, \"humidity_percent\": %.2f}", 
                           aht21_data.temperature_c, aht21_data.humidity_percent);
    } else {
        offset += snprintf(response + offset, sizeof(response) - offset, ", \"aht21\": {\"error\": %d}", aht21_result);
    }
    
    offset += snprintf(response + offset, sizeof(response) - offset, "}");
    
    // Send the response
    httpd_resp_set_type(request, "application/json");
    httpd_resp_sendstr(request, response);

    return ESP_OK;
}


// Function to start the HTTP server
void start_http_server(void) {
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    // Start the httpd server
    ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK) {
        ESP_LOGI(TAG, "Registering URI handlers");

        // Register URI handler for sensor data
        httpd_uri_t sensor_data_uri = {
            .uri      = "/sensor_data",
            .method   = HTTP_GET,
            .handler  = sensor_data_get_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &sensor_data_uri);
    } else {
        ESP_LOGI(TAG, "Error starting server!");
    }
}
