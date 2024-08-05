#include "sensors.h"
#include "led_control.h"

#include <driver/i2c.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_timer.h>
#include <math.h>

inline uint16_t combine_u16(uint8_t b0, uint8_t b1) {
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
    return ((uint16_t)b0 << 8) | b1;
#else
    return ((uint16_t)b1 << 8) | b0;
#endif
}

inline uint32_t combine_u32(uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3) {
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
    return ((uint32_t)b0 << 24) | ((uint32_t)b1 << 16) | ((uint32_t)b2 << 8) | b3;
#else
    return ((uint32_t)b3 << 24) | ((uint32_t)b2 << 16) | ((uint32_t)b1 << 8) | b0;
#endif
}

#define SENSORS_I2C_NUMBER I2C_NUM_0

#define ENS160_ADDRESS 0x53

#define ENS160_OPMODE_REGISTER 0x10
#define ENS160_OPMODE_STANDARD 0x02

#define ENS160_DATA_STATUS_REGISTER 0x20
#define ENS160_DATA_AQI 0x21
#define ENS160_DATA_TVOC_REGISTER 0x22
#define ENS160_DATA_ECO2_REGISTER 0x24
#define ENS160_DATA_T_REGISTER 0x30
#define ENS160_DATA_RH_REGISTER 0x32

#define AHT21_ADDRESS 0x38

static const char *TAG = "SENSORS";

#define STATUS_CHECK_INTERVAL_MS 1000 // Check every 5 seconds

static esp_err_t ens160_set_standard_mode(void) {
    uint8_t data = ENS160_OPMODE_STANDARD;
    i2c_cmd_handle_t command_handle = i2c_cmd_link_create();
    i2c_master_start(command_handle);
    i2c_master_write_byte(command_handle, (ENS160_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(command_handle, ENS160_OPMODE_REGISTER, true);  // Write to the OPMODE register
    i2c_master_write_byte(command_handle, data, true);  // Set to STANDARD mode
    i2c_master_stop(command_handle);
    esp_err_t result = i2c_master_cmd_begin(SENSORS_I2C_NUMBER, command_handle, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(command_handle);
    
    if (result == ESP_OK) {
        ESP_LOGI(TAG, "ENS160 set to STANDARD mode");
    } else {
        ESP_LOGE(TAG, "Failed to set ENS160 to STANDARD mode");
    }
    
    return result;
}

// Function to read an 8-bit register from the ENS160 sensor
static esp_err_t ens160_read_register_u8(uint8_t address, uint8_t *value) {
    uint8_t data = 0;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ENS160_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, address, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ENS160_ADDRESS << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, &data, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t result = i2c_master_cmd_begin(SENSORS_I2C_NUMBER, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);

    if (result == ESP_OK) {
        *value = data;
    } else {
        ESP_LOGE(TAG, "Failed to read register 0x%02X", address);
    }

    return result;
}

// Function to read a 16-bit register from the ENS160 sensor
static esp_err_t ens160_read_register_u16(uint8_t address, uint16_t *value) {
    uint8_t data[2] = {0};
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ENS160_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, address, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ENS160_ADDRESS << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, 2, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t result = i2c_master_cmd_begin(SENSORS_I2C_NUMBER, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);

    if (result == ESP_OK) {
        *value = combine_u16(data[0], data[1]);
    } else {
        ESP_LOGE(TAG, "Failed to read register 0x%02X", address);
    }

    return result;
}

esp_err_t ens160_read_data(struct ens160_data *data) {
    esp_err_t result;
    
    // Read AQI (Air Quality Index)
    result = ens160_read_register_u8(ENS160_DATA_AQI, &data->aqi);
    if (result != ESP_OK) {
        return result;
    } else {
        data->aqi &= 0x07;
    }
    
    // Read TVOC (Total Volatile Organic Compounds)
    result = ens160_read_register_u16(ENS160_DATA_TVOC_REGISTER, &data->tvoc_ppb);
    if (result != ESP_OK) {
        return result;
    }
    
    // Read eCO2 (Equivalent CO2 concentration)
    result = ens160_read_register_u16(ENS160_DATA_ECO2_REGISTER, &data->eco2_ppm);
    if (result != ESP_OK) {
        return result;
    }
    
    // Read Temperature
    uint16_t raw_temp;
    result = ens160_read_register_u16(ENS160_DATA_T_REGISTER, &raw_temp);
    if (result != ESP_OK) {
        return result;
    } else {
        data->temperature_c = (raw_temp / 64.0) - 273.15; // Convert to Celsius
    }
    
    // Read Relative Humidity
    uint16_t raw_humidity;
    result = ens160_read_register_u16(ENS160_DATA_RH_REGISTER, &raw_humidity);
    if (result != ESP_OK) {
        return result;
    } else {
        data->humidity_percent = raw_humidity / 512.0f; // Convert to percentage
    }
    
    ESP_LOGI(TAG, "ENS160 AQI: %d", data->aqi);
    ESP_LOGI(TAG, "ENS160 TVOC: %d ppb", data->tvoc_ppb);
    ESP_LOGI(TAG, "ENS160 eCO2: %d ppm", data->eco2_ppm);
    ESP_LOGI(TAG, "ENS160 Temperature: %.2f °C", data->temperature_c);
    ESP_LOGI(TAG, "ENS160 Humidity: %.2f %%", data->humidity_percent);
    
    return ESP_OK;
}

static esp_err_t ens160_sensor_status(void) {
    uint8_t data_status = 0;
    i2c_cmd_handle_t status_cmd = i2c_cmd_link_create();
    i2c_master_start(status_cmd);
    i2c_master_write_byte(status_cmd, (ENS160_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(status_cmd, ENS160_DATA_STATUS_REGISTER, true);
    i2c_master_start(status_cmd);
    i2c_master_write_byte(status_cmd, (ENS160_ADDRESS << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(status_cmd, &data_status, I2C_MASTER_LAST_NACK);
    i2c_master_stop(status_cmd);
    esp_err_t result = i2c_master_cmd_begin(SENSORS_I2C_NUMBER, status_cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(status_cmd);
    
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "ENS160 Status: Failed to read register!");
        led_set_status(status_led_channel, status_led_encoder, LED_STATUS_ERROR);  // Set LED to error (red)
        vTaskDelay(pdMS_TO_TICKS(STATUS_CHECK_INTERVAL_MS));
        return result;
    }
    
    // if (data_status & (1 << 0)) {
    //     ESP_LOGW(TAG, "ENS160 Status: GPR Data Ready");
    // }
    
    // if (data_status & (1 << 1)) {
    //     ESP_LOGW(TAG, "ENS160 Status: Data Ready");
    // }
    
    uint8_t status_flag = (data_status >> 2) & 0x3;
    switch (status_flag) {
        case 0:
            ESP_LOGI(TAG, "ENS160 Status: Operating ok");
            led_set_status(status_led_channel, status_led_encoder, LED_STATUS_NORMAL);  // Set LED to normal (green)
            break;
        case 1:
            ESP_LOGI(TAG, "ENS160 Status: Warm-up phase");
            led_set_status(status_led_channel, status_led_encoder, LED_STATUS_WARMUP);  // Set LED to warm-up (yellow)
            break;
        case 2:
            ESP_LOGI(TAG, "ENS160 Status: Initial Start-up (first hour)");
            led_set_status(status_led_channel, status_led_encoder, LED_STATUS_WARNING);  // Set LED to warning (orange)
            break;
        case 3:
            ESP_LOGI(TAG, "ENS160 Status: No valid output, multiple sensors out of range");
            led_set_status(status_led_channel, status_led_encoder, LED_STATUS_ERROR);  // Set LED to error (red)
            break;
    }
    
    if (data_status & (1 << 6)) {
        ESP_LOGW(TAG, "ENS160 Status: Error detected!");
    }
    
    // if (data_status & (1 << 7)) {
    //     ESP_LOGW(TAG, "ENS160 Status: OPMODE is running!");
    // }
    
    return ESP_OK;
}

#define AHT21_ADDRESS 0x38
#define AHT21_STATUS_CMD 0x71
#define AHT21_TRIGGER_MEASUREMENT_CMD 0xAC
#define AHT21_STATUS_COMPLETED 0x18
#define AHT21_INITIALIZE_REGISTER_1 0x1B
#define AHT21_INITIALIZE_REGISTER_2 0x1C
#define AHT21_INITIALIZE_REGISTER_3 0x1E

// Helper function to read a single byte from the sensor
static esp_err_t aht21_read_byte(uint8_t address, uint8_t *data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AHT21_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, address, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AHT21_ADDRESS << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, data, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(SENSORS_I2C_NUMBER, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t aht21_reset_register(uint8_t address) {
    uint8_t byte_second, byte_third;
    
    // First sequence: Read initial register values
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AHT21_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, address, true);
    i2c_master_write_byte(cmd, 0x00, true);
    i2c_master_write_byte(cmd, 0x00, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(SENSORS_I2C_NUMBER, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) return ret;

    vTaskDelay(pdMS_TO_TICKS(5)); // Delay 5ms

    // Read the values to reset the register
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AHT21_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x71, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AHT21_ADDRESS << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, NULL, I2C_MASTER_ACK); // Discard first byte
    i2c_master_read_byte(cmd, &byte_second, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, &byte_third, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(SENSORS_I2C_NUMBER, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) return ret;

    vTaskDelay(pdMS_TO_TICKS(10)); // Delay 10ms

    // Second sequence: Write the modified values back to the register
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AHT21_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0xB0 | address, true); // Command to reset register
    i2c_master_write_byte(cmd, byte_second, true);
    i2c_master_write_byte(cmd, byte_third, true);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(SENSORS_I2C_NUMBER, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);

    return ret;
}

// Function to initialize the AHT21 sensor
static esp_err_t aht21_initialize(void) {
    uint8_t status = 0;

    // Power on delay
    vTaskDelay(pdMS_TO_TICKS(200));

    // Read status word
    esp_err_t result = aht21_read_byte(AHT21_STATUS_CMD, &status);
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read status word");
        return result;
    }

    // Check if the status word is not equal to 0x18
    if ((status & 0x18) != AHT21_STATUS_COMPLETED) {
        ESP_LOGI(TAG, "AHT21 Sensor: Initializing registers...");
        aht21_reset_register(AHT21_INITIALIZE_REGISTER_1);
        aht21_reset_register(AHT21_INITIALIZE_REGISTER_2);
        aht21_reset_register(AHT21_INITIALIZE_REGISTER_3);
    }

    return ESP_OK;
}

// Function to trigger measurement and read data into aht21_data struct
esp_err_t aht21_read_data(struct aht21_data *data) {
    // Send the trigger measurement command
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AHT21_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, AHT21_TRIGGER_MEASUREMENT_CMD, true);
    i2c_master_write_byte(cmd, 0x33, true);
    i2c_master_write_byte(cmd, 0x00, true);
    i2c_master_stop(cmd);
    esp_err_t result = i2c_master_cmd_begin(SENSORS_I2C_NUMBER, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "Failed to trigger measurement");
        return result;
    }
    
    // Wait for measurement to complete
    vTaskDelay(pdMS_TO_TICKS(80));
    
    // Read the status word to check if measurement is complete
    uint8_t status;
    result = aht21_read_byte(AHT21_STATUS_CMD, &status);
    if (result != ESP_OK || (status & 0x80) != 0) {
        ESP_LOGE(TAG, "Measurement not completed");
        return result;
    }
    
    // Read 6 bytes of data
    uint8_t raw_data[7] = {0};
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AHT21_ADDRESS << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, raw_data, 7, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    result = i2c_master_cmd_begin(SENSORS_I2C_NUMBER, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read sensor data");
        return result;
    }
    
    // Convert data to temperature and humidity, two 20 bit packed integers:
    uint32_t raw_humidity = 0;
    raw_humidity |= raw_data[1] << 12;
    raw_humidity |= raw_data[2] << 4;
    raw_humidity |= raw_data[3] >> 4;
    
    uint32_t raw_temperature = 0;
    raw_temperature |= (raw_data[3] & 0xF) << 16;
    raw_temperature |= raw_data[4] << 8;
    raw_temperature |= raw_data[5];
    
    data->humidity_percent = ((float)raw_humidity / 0x100000) * 100;
    data->temperature_c = ((float)raw_temperature / 0x100000) * 200 - 50;
    
    ESP_LOGI(TAG, "AHT21 Humidity: %.2f %%", data->humidity_percent);
    ESP_LOGI(TAG, "AHT21 Temperature: %.2f °C", data->temperature_c);
    
    return ESP_OK;
}

static esp_err_t aht21_sensor_status(void) {
    uint8_t status = 0;
    esp_err_t result = aht21_read_byte(AHT21_STATUS_CMD, &status);
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read status word");
        return result;
    }
    
    if ((status & 0x80) == 0) {
        ESP_LOGI(TAG, "AHT21 Status: Measurement completed");
    } else {
        ESP_LOGI(TAG, "AHT21 Status: Measurement not completed");
    }
    
    return ESP_OK;
}

static esp_err_t sensors_i2c_master_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = GPIO_NUM_21,
        .scl_io_num = GPIO_NUM_22,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000,
    };

    esp_err_t err = i2c_param_config(SENSORS_I2C_NUMBER, &conf);
    if (err != ESP_OK) {
        ESP_LOGE("I2C_INIT", "Failed to configure I2C");
        return err;
    }

    err = i2c_driver_install(SENSORS_I2C_NUMBER, conf.mode, 0, 0, 0);
    if (err != ESP_OK) {
        ESP_LOGE("I2C_INIT", "Failed to install I2C driver");
        return err;
    }

    return ESP_OK;
}

void sensor_status_task(void *arguments) {
    while (1) {
        // ens160_sensor_status();
        // aht21_sensor_status();
        
        struct ens160_data ens160_data = {0};
        struct aht21_data aht21_data = {0};
        
        esp_err_t ens160_result = ens160_read_data(&ens160_data);
        esp_err_t aht21_result = aht21_read_data(&aht21_data);
        
        if (ens160_result != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read ENS160 data");
            led_set_status(status_led_channel, status_led_encoder, LED_STATUS_ERROR);
        }
        
        if (aht21_result != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read AHT21 data");
            led_set_status(status_led_channel, status_led_encoder, LED_STATUS_ERROR);
        }
        
        // Fade from green to red based on the air quality:
        float temperature_quality = fabs((aht21_data.temperature_c - 20.0f) / 10.0f);
        ESP_LOGI(TAG, "Temperature Quality: %.2f", temperature_quality);
        
        float humidity_quality = fabs((ens160_data.humidity_percent - 50.0f) / 20.0f);
        ESP_LOGI(TAG, "Humidity Quality: %.2f", humidity_quality);
        
        float tvoc_quality = ens160_data.tvoc_ppb / 300.0;
        ESP_LOGI(TAG, "TVOC Quality: %.2f", tvoc_quality);
        
        float eco2_quality = ens160_data.eco2_ppm / 1000.0;
        ESP_LOGI(TAG, "eCO2 Quality: %.2f", eco2_quality);
        
        float quality = (temperature_quality + humidity_quality + tvoc_quality + eco2_quality) / 4.0;
        ESP_LOGI(TAG, "Quality: %.2f", quality);
        if (quality < 0.0) quality = 0.0;
        if (quality > 1.0) quality = 1.0;
        
        // 0 = good, 1 = bad
        led_set_color(status_led_channel, status_led_encoder, 255 * quality, 255 * (1.0 - quality), 0);
        
        // Wait for the next cycle
        vTaskDelay(pdMS_TO_TICKS(STATUS_CHECK_INTERVAL_MS));
    }
}

void sensors_initialize(void) {
    esp_err_t error;
    
    error = sensors_i2c_master_init();
    if (error != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize I2C");
        led_set_status(status_led_channel, status_led_encoder, LED_STATUS_ERROR);
        return;
    }
    
    error = ens160_set_standard_mode();
    if (error != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set ENS160 to STANDARD mode");
        led_set_status(status_led_channel, status_led_encoder, LED_STATUS_ERROR);
        return;
    }
    
    error = aht21_initialize();
    if (error != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize AHT21");
        led_set_status(status_led_channel, status_led_encoder, LED_STATUS_ERROR);
        return;
    }
    
    xTaskCreate(&sensor_status_task, "Sensor Status Task", 2048, NULL, 5, NULL);
}
