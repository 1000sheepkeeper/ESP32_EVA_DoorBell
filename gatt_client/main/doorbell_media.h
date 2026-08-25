/*
 * Doorbell media front-end.
 *
 * The BLE client calls doorbell_media_trigger() from its notification callback.
 * The function only queues the event; SD-card access, WAV playback and LVGL
 * animation changes are performed by the media task.
 */
#pragma once

#include <stdint.h>

#include "esp_err.h"

/** Initialise the ST7735, mount the SD card and start the idle UI task. */
esp_err_t doorbell_media_init(void);

/** Queue a doorbell event. event_id is the first byte of the BLE notification. */
esp_err_t doorbell_media_trigger(uint8_t event_id);
