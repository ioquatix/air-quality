#pragma once

#include <stdint.h>
#include <esp_err.h>

struct ens160_data {
    uint8_t aqi;
    uint16_t tvoc_ppb;      // Total Volatile Organic Compounds (ppb)
    uint16_t eco2_ppm;      // Equivalent CO2 concentration (ppm)
    float temperature_c;    // Temperature in Celsius
    float humidity_percent; // Relative Humidity in %
};

struct aht21_data {
		float temperature_c;    // Temperature in Celsius
		float humidity_percent; // Relative Humidity in %
};

void sensors_initialize(void);

esp_err_t ens160_read_data(struct ens160_data *data);
esp_err_t aht21_read_data(struct aht21_data *data);
