// led_control.c

#include "led_control.h"
#include "esp_log.h"

#define TAG "LED_CONTROL"

// Define the GPIO pin for the WS2812
#define WS2812_GPIO GPIO_NUM_8

// Define the timing for the WS2812 signals
#define WS2812_T0H_NS 350  // 0 bit high time in nanoseconds
#define WS2812_T0L_NS 800  // 0 bit low time in nanoseconds
#define WS2812_T1H_NS 700  // 1 bit high time in nanoseconds
#define WS2812_T1L_NS 600  // 1 bit low time in nanoseconds

rmt_channel_handle_t status_led_channel = NULL;
rmt_encoder_handle_t status_led_encoder = NULL;

// Define the RGB color structure
typedef struct {
    uint8_t green;
    uint8_t red;
    uint8_t blue;
} ws2812_color_t;

// Initialize the RMT TX channel and encoder
rmt_channel_handle_t led_initialize(rmt_encoder_handle_t *encoder) {
    rmt_tx_channel_config_t tx_chan_config = {
        .gpio_num = WS2812_GPIO,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10000000,  // 10MHz resolution (100ns period)
        .mem_block_symbols = 64,    // Memory block size
        .trans_queue_depth = 4,     // Transaction queue depth
    };

    rmt_channel_handle_t tx_chan = NULL;
    esp_err_t err;

    if ((err = rmt_new_tx_channel(&tx_chan_config, &tx_chan)) != ESP_OK) {
        ESP_LOGE(TAG, "rmt_new_tx_channel failed: %s", esp_err_to_name(err));
        return NULL;
    }

    if ((err = rmt_enable(tx_chan)) != ESP_OK) {
        ESP_LOGE(TAG, "rmt_enable failed: %s", esp_err_to_name(err));
        return NULL;
    }

    rmt_copy_encoder_config_t copy_encoder_config = {};
    if ((err = rmt_new_copy_encoder(&copy_encoder_config, encoder)) != ESP_OK) {
        ESP_LOGE(TAG, "rmt_new_copy_encoder failed: %s", esp_err_to_name(err));
        return NULL;
    }

    return tx_chan;
}

// Set the LED color
void led_set_color(rmt_channel_handle_t tx_chan, rmt_encoder_handle_t encoder, uint8_t red, uint8_t green, uint8_t blue) {
    ws2812_color_t led_color = { .red = red, .green = green, .blue = blue };

    uint32_t color = (led_color.green << 16) | (led_color.red << 8) | led_color.blue;
    rmt_symbol_word_t items[24];

    for (int bit = 23; bit >= 0; bit--) {
        if (color & (1 << bit)) {
            items[23 - bit].duration0 = WS2812_T1H_NS / 100;
            items[23 - bit].level0 = 1;
            items[23 - bit].duration1 = WS2812_T1L_NS / 100;
            items[23 - bit].level1 = 0;
        } else {
            items[23 - bit].duration0 = WS2812_T0H_NS / 100;
            items[23 - bit].level0 = 1;
            items[23 - bit].duration1 = WS2812_T0L_NS / 100;
            items[23 - bit].level1 = 0;
        }
    }

    rmt_transmit_config_t tx_config = {
        .loop_count = 0,  // No looping
    };

    rmt_transmit(tx_chan, encoder, items, sizeof(items), &tx_config);
}

void led_set_status(rmt_channel_handle_t tx_chan, rmt_encoder_handle_t encoder, int status) {
    switch (status) {
        case LED_STATUS_WARMUP:
            led_set_color(tx_chan, encoder, 255, 255, 0);  // Yellow for warmup
            break;
        case LED_STATUS_NORMAL:
            led_set_color(tx_chan, encoder, 0, 255, 0);    // Green for normal operation
            break;
        case LED_STATUS_WARNING:
            led_set_color(tx_chan, encoder, 255, 165, 0);  // Orange for warning
            break;
        case LED_STATUS_ERROR:
            led_set_color(tx_chan, encoder, 255, 0, 0);    // Red for error
            break;
        default:
            led_set_color(tx_chan, encoder, 0, 0, 0);      // Turn off LED for unknown status
            break;
    }
}
