#pragma once

#include "driver/rmt_tx.h"

extern rmt_channel_handle_t status_led_channel;
extern rmt_encoder_handle_t status_led_encoder;

enum led_status {
	LED_STATUS_WARMUP = 0,
	LED_STATUS_NORMAL = 1,
	LED_STATUS_WARNING = 2,
	LED_STATUS_ERROR = 3
};

// Initialize the LED control
rmt_channel_handle_t led_initialize(rmt_encoder_handle_t *encoder);

// Set the LED color
void led_set_color(rmt_channel_handle_t tx_chan, rmt_encoder_handle_t encoder, uint8_t red, uint8_t green, uint8_t blue);
void led_set_status(rmt_channel_handle_t tx_chan, rmt_encoder_handle_t encoder, int status);
