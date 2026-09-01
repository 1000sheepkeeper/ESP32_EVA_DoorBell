/*
 * SPDX-FileCopyrightText: 2025
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 *
 * Memory-based WAV player over I2S (MAX98357A style, 16-bit stereo Philips).
 *
 * Audio clips are embedded in flash as C arrays (see wav_assets.h).
 * Supported input: PCM 16-bit, mono (duplicated to both channels) or stereo.
 * The clip's own sample rate is honoured (I2S is re-initialised per clip).
 *
 * Volume: attenuation level 0..8 applied as an arithmetic right-shift
 * on every sample (~6 dB per step).
 */

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/i2s_std.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "audio_player.h"
#include "wav_assets.h"

#define TAG "audio_player"

/* ── Hardware pins (verified with the i2s_example project on ESP32) ── */
#define I2S_BCLK_GPIO   GPIO_NUM_4
#define I2S_WS_GPIO     GPIO_NUM_5
#define I2S_DOUT_GPIO   GPIO_NUM_18

/* ── Player configuration ── */
#define PLAY_QUEUE_DEPTH    3
#define PLAY_TASK_PRIO      10
#define PLAY_TASK_STACK     4096
#define CHUNK_SAMPLES       1024        /* samples (per channel) per I2S write */
#define DRAIN_SILENCE_MS    30

/* The embedded doorbell.wav peaks at about -15 dBFS, so apply a fixed
 * +12 dB software boost (level 0 = +12 dB, each step ~6 dB quieter).
 * Boost <<2 was verified to not clip any sample of the clip. */
#define VOL_BOOST_SHIFT     2
#define VOL_EFF_DB(level)   (((VOL_BOOST_SHIFT) - (level)) * 6)

static i2s_chan_handle_t tx_chan = NULL;
static QueueHandle_t play_queue = NULL;
static int s_vol_level = 0;             /* default: full volume (+12 dB boost) */

typedef struct {
    const uint8_t *data;
    uint32_t       len;
} clip_t;

static const clip_t clip_doorbell = {
    .data = doorbell_wav,
    .len  = doorbell_wav_len,
};

typedef struct {
    uint32_t sample_rate;
    uint16_t channels;                  /* 1 or 2 */
    uint32_t data_off;
    uint32_t data_len;
} wav_info_t;

static inline uint16_t rd16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static inline uint32_t rd32(const uint8_t *p) { return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
                                                       ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24); }

/* Parse RIFF/WAVE header from a memory buffer (PCM 16-bit only). */
static esp_err_t wav_parse_mem(const uint8_t *buf, uint32_t len, wav_info_t *out)
{
    if (!buf || !out || len < 44) {
        return ESP_ERR_INVALID_ARG;
    }
    if (memcmp(buf, "RIFF", 4) != 0 || memcmp(buf + 8, "WAVE", 4) != 0) {
        ESP_LOGE(TAG, "Not a RIFF/WAVE file");
        return ESP_ERR_INVALID_ARG;
    }
    if (rd16(buf + 20) != 1) {          /* audio format: PCM */
        ESP_LOGE(TAG, "Only PCM supported");
        return ESP_ERR_NOT_SUPPORTED;
    }
    uint16_t bits = rd16(buf + 34);
    if (bits != 16) {
        ESP_LOGE(TAG, "Only 16-bit supported (got %u)", bits);
        return ESP_ERR_NOT_SUPPORTED;
    }

    out->channels    = rd16(buf + 22);
    out->sample_rate = rd32(buf + 24);
    if (out->channels != 1 && out->channels != 2) {
        ESP_LOGE(TAG, "Unsupported channel count %u", out->channels);
        return ESP_ERR_NOT_SUPPORTED;
    }

    /* Scan chunks for "data" */
    uint32_t off = 12;
    while (off + 8 <= len) {
        uint32_t sz = rd32(buf + off + 4);
        if (memcmp(buf + off, "data", 4) == 0) {
            out->data_off = off + 8;
            out->data_len = sz;
            if (out->data_off + out->data_len > len) {
                out->data_len = len - out->data_off;
            }
            return ESP_OK;
        }
        off += 8 + sz + (sz & 1);
    }
    ESP_LOGE(TAG, "No data chunk found");
    return ESP_ERR_INVALID_ARG;
}

/* Initialise I2S TX channel (16-bit, stereo, Philips) for the given rate. */
static void i2s_tx_init(uint32_t sample_rate)
{
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
    chan_cfg.dma_desc_num  = 12;
    chan_cfg.dma_frame_num = 480;
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &tx_chan, NULL));

    i2s_std_config_t std_cfg = {
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(sample_rate),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT,
                                                        I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = I2S_BCLK_GPIO,
            .ws   = I2S_WS_GPIO,
            .dout = I2S_DOUT_GPIO,
            .din  = I2S_GPIO_UNUSED,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv   = false,
            },
        },
    };
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx_chan, &std_cfg));
    ESP_LOGI(TAG, "I2S TX ready: %lu Hz, 16-bit, stereo", (unsigned long)sample_rate);
}

/* Play one embedded clip to completion. Called from the playback task only. */
static void play_clip(const clip_t *clip)
{
    wav_info_t wav = {0};
    if (wav_parse_mem(clip->data, clip->len, &wav) != ESP_OK) {
        ESP_LOGE(TAG, "Bad WAV clip (%u bytes)", (unsigned)clip->len);
        return;
    }

    ESP_LOGI(TAG, "Playing clip: %lu Hz, %u ch, %u bytes (vol level %d)",
             (unsigned long)wav.sample_rate, wav.channels,
             (unsigned)wav.data_len, s_vol_level);

    i2s_tx_init(wav.sample_rate);
    ESP_ERROR_CHECK(i2s_channel_enable(tx_chan));

    const int16_t *in = (const int16_t *)(clip->data + wav.data_off);
    const uint32_t total_samples = wav.data_len / 2;    /* interleaved samples */
    static int16_t out[CHUNK_SAMPLES * 2];

    uint32_t done = 0;
    while (done < total_samples) {
        uint32_t todo = total_samples - done;
        if (todo > CHUNK_SAMPLES) {
            todo = CHUNK_SAMPLES;
        }

        int shift = s_vol_level - VOL_BOOST_SHIFT;      /* snapshot per chunk; negative = boost */
        if (wav.channels == 1) {
            for (uint32_t i = 0; i < todo; i++) {
                int32_t v = in[done + i];
                if (shift >= 0) {
                    v >>= shift;
                } else {
                    v <<= -shift;
                    if (v > INT16_MAX) { v = INT16_MAX; }
                    else if (v < INT16_MIN) { v = INT16_MIN; }
                }
                int16_t s = (int16_t)v;
                out[i * 2]     = s;
                out[i * 2 + 1] = s;
            }
        } else {
            for (uint32_t i = 0; i < todo; i++) {
                int32_t l = in[(done + i) * 2];
                int32_t r = in[(done + i) * 2 + 1];
                if (shift >= 0) {
                    l >>= shift;
                    r >>= shift;
                } else {
                    l <<= -shift;
                    r <<= -shift;
                    if (l > INT16_MAX) { l = INT16_MAX; }
                    else if (l < INT16_MIN) { l = INT16_MIN; }
                    if (r > INT16_MAX) { r = INT16_MAX; }
                    else if (r < INT16_MIN) { r = INT16_MIN; }
                }
                out[i * 2]     = (int16_t)l;
                out[i * 2 + 1] = (int16_t)r;
            }
        }

        size_t written = 0;
        ESP_ERROR_CHECK(i2s_channel_write(tx_chan, out, todo * 2 * sizeof(int16_t),
                                          &written, portMAX_DELAY));
        done += todo;
    }

    /* Drain the DMA pipeline with silence so the tail isn't cut off */
    static int16_t silence[CHUNK_SAMPLES * 2] = {0};
    size_t written = 0;
    ESP_ERROR_CHECK(i2s_channel_write(tx_chan, silence, sizeof(silence),
                                      &written, portMAX_DELAY));
    vTaskDelay(pdMS_TO_TICKS(DRAIN_SILENCE_MS));

    ESP_ERROR_CHECK(i2s_channel_disable(tx_chan));
    ESP_ERROR_CHECK(i2s_del_channel(tx_chan));
    tx_chan = NULL;
    ESP_LOGI(TAG, "Clip finished");
}

/* Playback task: waits for clip requests and plays them one at a time. */
static void playback_task(void *arg)
{
    while (1) {
        clip_t clip;
        if (xQueueReceive(play_queue, &clip, portMAX_DELAY) == pdTRUE) {
            play_clip(&clip);
        }
    }
}

/* ── Public API ── */

esp_err_t audio_player_init(void)
{
    if (play_queue) {
        return ESP_OK;                  /* already initialised */
    }
    play_queue = xQueueCreate(PLAY_QUEUE_DEPTH, sizeof(clip_t));
    if (!play_queue) {
        ESP_LOGE(TAG, "Failed to create play queue");
        return ESP_ERR_NO_MEM;
    }
    if (xTaskCreate(playback_task, "audio_playback", PLAY_TASK_STACK, NULL,
                    PLAY_TASK_PRIO, NULL) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create playback task");
        vQueueDelete(play_queue);
        play_queue = NULL;
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "Audio player ready, volume level %d (effective gain %+d dB)",
             s_vol_level, VOL_EFF_DB(s_vol_level));
    return ESP_OK;
}

esp_err_t audio_player_play_doorbell(void)
{
    if (!play_queue) {
        ESP_LOGW(TAG, "Player not initialised");
        return ESP_ERR_INVALID_STATE;
    }
    if (xQueueSend(play_queue, &clip_doorbell, 0) != pdTRUE) {
        ESP_LOGW(TAG, "Play queue full, doorbell clip dropped");
        return ESP_FAIL;
    }
    return ESP_OK;
}

void audio_player_volume_up(void)
{
    if (s_vol_level > 0) {
        s_vol_level--;
    }
    ESP_LOGI(TAG, "Volume up -> level %d (effective gain %+d dB)", s_vol_level, VOL_EFF_DB(s_vol_level));
}

void audio_player_volume_down(void)
{
    if (s_vol_level < AUDIO_VOLUME_LEVELS - 1) {
        s_vol_level++;
    }
    ESP_LOGI(TAG, "Volume down -> level %d (effective gain %+d dB)", s_vol_level, VOL_EFF_DB(s_vol_level));
}

int audio_player_get_volume_level(void)
{
    return s_vol_level;
}
