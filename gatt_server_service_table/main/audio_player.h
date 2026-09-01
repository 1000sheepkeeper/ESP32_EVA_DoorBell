/*
 * SPDX-FileCopyrightText: 2025
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 *
 * Audio player: plays WAV clips embedded in flash through I2S (MAX98357A).
 * Volume is a per-sample right-shift attenuation, adjustable at runtime.
 */

#pragma once

#include <stdint.h>
#include "esp_err.h"

/* Volume levels 0..8: level 0 = full volume with a fixed +12 dB software
 * boost (the embedded clip is quiet), each step is ~6 dB quieter. */
#define AUDIO_VOLUME_LEVELS 9

/**
 * @brief Initialise audio player (creates playback task and request queue).
 * @return ESP_OK on success
 */
esp_err_t audio_player_init(void);

/**
 * @brief Queue a doorbell clip for playback (non-blocking).
 * @return ESP_OK if queued, ESP_ERR_INVALID_STATE if player not initialised,
 *         ESP_FAIL if the queue is full (request dropped).
 */
esp_err_t audio_player_play_doorbell(void);

/** @brief Increase volume (decrease attenuation). */
void audio_player_volume_up(void);

/** @brief Decrease volume (increase attenuation). */
void audio_player_volume_down(void);

/** @brief Current attenuation level (0 = loudest). */
int audio_player_get_volume_level(void);
