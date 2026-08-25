/*
 * ST7735 doorbell home screen for ESP32.
 *
 * Home background is embedded in the firmware.  The character animations and
 * the doorbell WAV are read from the SD-card root directory at run time.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2s_std.h"
#include "driver/spi_master.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_lcd_panel_io.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "lvgl.h"

#include "doorbell_media.h"

static const char *TAG = "doorbell_home";

/* SD card wiring, copied from /home/sk/espprj/i2s_example. */
#define SD_MOSI_GPIO        26
#define SD_MISO_GPIO        19
#define SD_SCLK_GPIO        25
#define SD_CS_GPIO          27

/* ST7735 wiring. */
#define TFT_SCLK_GPIO       14
#define TFT_MOSI_GPIO       13
#define TFT_RST_GPIO        21
#define TFT_DC_GPIO         22
#define TFT_CS_GPIO         32
#define TFT_BL_GPIO         33
#define TFT_BL_ON_LEVEL     1

/* I2S amplifier/DAC wiring. */
#define AUDIO_BCLK_GPIO      GPIO_NUM_4
#define AUDIO_WS_GPIO        GPIO_NUM_5
#define AUDIO_DOUT_GPIO      GPIO_NUM_18

#define TFT_WIDTH            128
#define TFT_HEIGHT           160
#define TFT_X_OFFSET         3
#define TFT_Y_OFFSET         2
#define TFT_SPI_HOST         SPI3_HOST

#define MOUNT_POINT          "/sdcard"
#define IDLE_GIF_PATH        "S:/idle.gif"
#define KNOCK_GIF_PATH       "S:/knock.gif"
#define DOORBELL_WAV_PATH    MOUNT_POINT "/doorbell.wav"

#define LVGL_DRAW_LINES      20
#define CHARACTER_WIDTH      88
#define CHARACTER_HEIGHT     104
#define AUDIO_INPUT_BYTES    2048
#define AUDIO_FILE_CACHE_BYTES 4096

#define CMD_SWRESET          0x01
#define CMD_SLPOUT           0x11
#define CMD_NORON            0x13
#define CMD_INVOFF           0x20
#define CMD_DISPON           0x29
#define CMD_CASET            0x2A
#define CMD_RASET            0x2B
#define CMD_RAMWR            0x2C
#define CMD_MADCTL           0x36
#define CMD_COLMOD           0x3A

/* Flip both row and column scan directions relative to the previous 0xC0
 * setting, rotating the complete 128x160 image by 180 degrees in hardware. */
#define TFT_MADCTL_ROTATION_180 0x00

static esp_lcd_panel_io_handle_t lcd_io;
static uint8_t lvgl_draw_buf1[TFT_WIDTH * LVGL_DRAW_LINES * 2] __attribute__((aligned(4)));
static uint8_t lvgl_draw_buf2[TFT_WIDTH * LVGL_DRAW_LINES * 2] __attribute__((aligned(4)));

/* The supplied 128x160 background, pre-converted to native RGB565. */
extern const uint8_t background_rgb565_start[] asm("_binary_background_rgb565_start");
extern const uint8_t background_rgb565_end[] asm("_binary_background_rgb565_end");
static lv_image_dsc_t background_rgb565;
static QueueHandle_t media_trigger_queue;
static volatile bool media_initialized;

typedef struct {
    uint32_t sample_rate;
    uint32_t data_size;
    uint32_t byte_rate;
    long data_offset;
    uint16_t channels;
    uint16_t bits_per_sample;
} wav_info_t;

static lv_obj_t *character_gif;
static volatile bool doorbell_audio_done = true;
static bool doorbell_sequence_active;
static bool knock_animation_done;

static uint8_t audio_input[AUDIO_INPUT_BYTES] __attribute__((aligned(4)));
static int16_t audio_output[AUDIO_INPUT_BYTES] __attribute__((aligned(4)));
static uint8_t audio_file_cache[AUDIO_FILE_CACHE_BYTES] __attribute__((aligned(4)));

static uint16_t read_le16(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

static uint32_t read_le32(const uint8_t *data)
{
    return (uint32_t)data[0] | ((uint32_t)data[1] << 8) |
           ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
}

static void lcd_cmd(uint8_t command, const uint8_t *data, size_t data_len)
{
    ESP_ERROR_CHECK(esp_lcd_panel_io_tx_param(lcd_io, command, data, data_len));
}

static void lcd_set_window(int x, int y, int width, int height)
{
    x += TFT_X_OFFSET;
    y += TFT_Y_OFFSET;
    lcd_cmd(CMD_CASET, (uint8_t[]){x >> 8, x, (x + width - 1) >> 8, x + width - 1}, 4);
    lcd_cmd(CMD_RASET, (uint8_t[]){y >> 8, y, (y + height - 1) >> 8, y + height - 1}, 4);
}

static void lcd_reset_and_init(void)
{
    gpio_set_level(TFT_RST_GPIO, 0);
    vTaskDelay(pdMS_TO_TICKS(20));
    gpio_set_level(TFT_RST_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(120));

    lcd_cmd(CMD_SWRESET, NULL, 0);
    vTaskDelay(pdMS_TO_TICKS(150));
    lcd_cmd(CMD_SLPOUT, NULL, 0);
    vTaskDelay(pdMS_TO_TICKS(120));

    lcd_cmd(0xB1, (uint8_t[]){0x01, 0x2C, 0x2D}, 3);
    lcd_cmd(0xB2, (uint8_t[]){0x01, 0x2C, 0x2D}, 3);
    lcd_cmd(0xB3, (uint8_t[]){0x01, 0x2C, 0x2D, 0x01, 0x2C, 0x2D}, 6);
    lcd_cmd(0xB4, (uint8_t[]){0x07}, 1);
    lcd_cmd(0xC0, (uint8_t[]){0xA2, 0x02, 0x84}, 3);
    lcd_cmd(0xC1, (uint8_t[]){0xC5}, 1);
    lcd_cmd(0xC2, (uint8_t[]){0x0A, 0x00}, 2);
    lcd_cmd(0xC3, (uint8_t[]){0x8A, 0x2A}, 2);
    lcd_cmd(0xC4, (uint8_t[]){0x8A, 0xEE}, 2);
    lcd_cmd(0xC5, (uint8_t[]){0x0E}, 1);
    lcd_cmd(CMD_INVOFF, NULL, 0);
    lcd_cmd(CMD_MADCTL, (uint8_t[]){TFT_MADCTL_ROTATION_180}, 1);
    lcd_cmd(CMD_COLMOD, (uint8_t[]){0x05}, 1);
    lcd_cmd(0xE0, (uint8_t[]){0x02,0x1C,0x07,0x12,0x37,0x32,0x29,0x2D,
                               0x29,0x25,0x2B,0x39,0x00,0x01,0x03,0x10}, 16);
    lcd_cmd(0xE1, (uint8_t[]){0x03,0x1D,0x07,0x06,0x2E,0x2C,0x29,0x2D,
                               0x2E,0x2E,0x37,0x3F,0x00,0x00,0x02,0x10}, 16);
    lcd_cmd(CMD_NORON, NULL, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    lcd_cmd(CMD_DISPON, NULL, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
}

/* LVGL has RGB565 pixels in host byte order; ST7735 needs high byte first. */
static void display_flush_cb(lv_display_t *display, const lv_area_t *area, uint8_t *px_map)
{
    int width = area->x2 - area->x1 + 1;
    int height = area->y2 - area->y1 + 1;
    size_t byte_count = (size_t)width * height * 2;

    for (size_t index = 0; index < byte_count; index += 2) {
        uint8_t low = px_map[index];
        px_map[index] = px_map[index + 1];
        px_map[index + 1] = low;
    }

    lcd_set_window(area->x1, area->y1, width, height);
    ESP_ERROR_CHECK(esp_lcd_panel_io_tx_color(lcd_io, CMD_RAMWR, px_map, byte_count));
    lcd_cmd(0x00, NULL, 0); /* Wait for the queued transfer before reuse. */
    lv_display_flush_ready(display);
}

static void lvgl_tick_callback(void *arg)
{
    (void)arg;
    lv_tick_inc(2);
}

static void character_gif_event_cb(lv_event_t *event)
{
    if (lv_event_get_code(event) == LV_EVENT_READY && doorbell_sequence_active) {
        knock_animation_done = true;
        ESP_LOGI(TAG, "Knock animation completed two loops");
    }
}

static bool set_character_animation(const char *path, int32_t loop_count)
{
    lv_gif_set_src(character_gif, path);
    if (!lv_gif_is_loaded(character_gif)) {
        ESP_LOGE(TAG, "Cannot load character animation: %s", path);
        return false;
    }

    lv_gif_restart(character_gif);
    /* lv_gif_restart resets the loop counter, so set it afterwards. */
    lv_gif_set_loop_count(character_gif, loop_count);
    return true;
}

static void create_home_ui(lv_display_t *display)
{
    lv_obj_t *screen = lv_display_get_screen_active(display);
    lv_obj_set_style_bg_color(screen, lv_color_black(), 0);
    lv_obj_set_style_pad_all(screen, 0, 0);
    lv_obj_set_scrollbar_mode(screen, LV_SCROLLBAR_MODE_OFF);

    lv_obj_t *background = lv_image_create(screen);
    lv_obj_set_size(background, TFT_WIDTH, TFT_HEIGHT);
    lv_image_set_src(background, &background_rgb565);
    lv_image_set_inner_align(background, LV_IMAGE_ALIGN_CENTER);

    character_gif = lv_gif_create(screen);
    lv_obj_set_size(character_gif, CHARACTER_WIDTH, CHARACTER_HEIGHT);
    lv_obj_center(character_gif);
    lv_image_set_inner_align(character_gif, LV_IMAGE_ALIGN_CONTAIN);
    lv_obj_add_event_cb(character_gif, character_gif_event_cb, LV_EVENT_READY, NULL);

    /* Zero means infinite playback in LVGL's GIF decoder. */
    if (!set_character_animation(IDLE_GIF_PATH, 0)) {
        lv_obj_t *message = lv_label_create(screen);
        lv_label_set_text(message, "INSERT idle.gif");
        lv_obj_set_width(message, TFT_WIDTH);
        lv_obj_set_style_text_align(message, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_set_style_text_color(message, lv_color_white(), 0);
        lv_obj_center(message);
    }
}

static esp_err_t mount_sdcard(sdmmc_card_t **card)
{
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.max_freq_khz = 8000; /* 8 MHz: SD SPI bus clock (20/10 MHz failed on this wiring) */
    spi_bus_config_t bus = {
        .mosi_io_num = SD_MOSI_GPIO,
        .miso_io_num = SD_MISO_GPIO,
        .sclk_io_num = SD_SCLK_GPIO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096,
    };
    ESP_RETURN_ON_ERROR(spi_bus_initialize(host.slot, &bus, SPI_DMA_CH_AUTO), TAG,
                        "SD SPI bus init failed");

    sdspi_device_config_t slot = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot.gpio_cs = SD_CS_GPIO;
    slot.host_id = host.slot;
    esp_vfs_fat_sdmmc_mount_config_t mount = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 4096,
    };
    esp_err_t result = esp_vfs_fat_sdspi_mount(MOUNT_POINT, &host, &slot, &mount, card);
    if (result != ESP_OK) {
        spi_bus_free(host.slot);
    }
    return result;
}

/* Read doorbell.wav end-to-end once and log the achieved SPI throughput. */
static void measure_sd_read_speed(void)
{
    FILE *file = fopen(DOORBELL_WAV_PATH, "rb");
    if (file == NULL) {
        ESP_LOGW(TAG, "Speed test skipped: cannot open %s", DOORBELL_WAV_PATH);
        return;
    }
    int64_t start_us = esp_timer_get_time();
    size_t read_total = 0;
    while (true) {
        size_t n = fread(audio_file_cache, 1, sizeof(audio_file_cache), file);
        if (n == 0) {
            break;
        }
        read_total += n;
    }
    int64_t elapsed_us = esp_timer_get_time() - start_us;
    fclose(file);

    if (elapsed_us <= 0 || read_total == 0) {
        ESP_LOGW(TAG, "Speed test failed: read %u bytes in %lld us",
                 (unsigned)read_total, (long long)elapsed_us);
        return;
    }
    uint32_t kbps = (uint32_t)(((uint64_t)read_total * 1000000ULL) / (uint64_t)elapsed_us / 1024ULL);
    ESP_LOGI(TAG, "SD read speed: %ld KB read in %lld ms -> %u KB/s (%.2f MB/s, SPI 20 MHz)",
             (long)(read_total / 1024), (long long)(elapsed_us / 1000), kbps, kbps / 1024.0f);
}

static esp_err_t parse_wav(FILE *file, wav_info_t *info)
{
    uint8_t riff_header[12];
    if (fread(riff_header, 1, sizeof(riff_header), file) != sizeof(riff_header) ||
        memcmp(riff_header, "RIFF", 4) != 0 || memcmp(riff_header + 8, "WAVE", 4) != 0) {
        return ESP_ERR_INVALID_ARG;
    }

    bool format_found = false;
    while (true) {
        uint8_t chunk_header[8];
        if (fread(chunk_header, 1, sizeof(chunk_header), file) != sizeof(chunk_header)) {
            return ESP_FAIL;
        }
        uint32_t chunk_size = read_le32(chunk_header + 4);
        long padded_size = (long)((chunk_size + 1U) & ~1U);

        if (memcmp(chunk_header, "fmt ", 4) == 0) {
            if (chunk_size < 16) {
                return ESP_ERR_INVALID_ARG;
            }
            uint8_t format[16];
            if (fread(format, 1, sizeof(format), file) != sizeof(format)) {
                return ESP_FAIL;
            }
            uint16_t audio_format = read_le16(format);
            info->channels = read_le16(format + 2);
            info->sample_rate = read_le32(format + 4);
            info->byte_rate = read_le32(format + 8);
            info->bits_per_sample = read_le16(format + 14);
            if (audio_format != 1 || info->bits_per_sample != 16 ||
                (info->channels != 1 && info->channels != 2) || info->sample_rate == 0) {
                return ESP_ERR_NOT_SUPPORTED;
            }
            if (info->byte_rate == 0) {
                info->byte_rate = info->sample_rate * info->channels * sizeof(int16_t);
            }
            padded_size -= sizeof(format);
            if (padded_size > 0 && fseek(file, padded_size, SEEK_CUR) != 0) {
                return ESP_FAIL;
            }
            format_found = true;
        } else if (memcmp(chunk_header, "data", 4) == 0) {
            if (!format_found) {
                return ESP_ERR_INVALID_STATE;
            }
            info->data_offset = ftell(file);
            info->data_size = chunk_size;
            return ESP_OK;
        } else if (fseek(file, padded_size, SEEK_CUR) != 0) {
            return ESP_FAIL;
        }
    }
}

static esp_err_t play_doorbell_wav(void)
{
    FILE *file = fopen(DOORBELL_WAV_PATH, "rb");
    if (file == NULL) {
        ESP_LOGE(TAG, "Cannot open %s", DOORBELL_WAV_PATH);
        return ESP_ERR_NOT_FOUND;
    }
    setvbuf(file, (char *)audio_file_cache, _IOFBF, sizeof(audio_file_cache));

    wav_info_t info = {0};
    esp_err_t result = parse_wav(file, &info);
    if (result != ESP_OK || fseek(file, info.data_offset, SEEK_SET) != 0) {
        ESP_LOGE(TAG, "doorbell.wav must be 16-bit PCM mono or stereo WAV");
        fclose(file);
        return result == ESP_OK ? ESP_FAIL : result;
    }

    i2s_chan_handle_t channel = NULL;
    i2s_chan_config_t channel_config = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
    channel_config.dma_desc_num = 12;
    channel_config.dma_frame_num = 480;
    channel_config.auto_clear_after_cb = true;
    channel_config.intr_priority = 3;
    result = i2s_new_channel(&channel_config, &channel, NULL);
    if (result != ESP_OK) {
        fclose(file);
        return result;
    }

    i2s_std_config_t config = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(info.sample_rate),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT,
                                                        I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = AUDIO_BCLK_GPIO,
            .ws = AUDIO_WS_GPIO,
            .dout = AUDIO_DOUT_GPIO,
            .din = I2S_GPIO_UNUSED,
            .invert_flags = {0},
        },
    };
    result = i2s_channel_init_std_mode(channel, &config);
    if (result == ESP_OK) {
        result = i2s_channel_enable(channel);
    }
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "I2S init failed: %s", esp_err_to_name(result));
        i2s_del_channel(channel);
        fclose(file);
        return result;
    }

    uint32_t remaining = info.data_size;
    size_t input_frame_bytes = info.channels * sizeof(int16_t);
    ESP_LOGI(TAG, "Playing doorbell: %lu Hz, %u channel(s)",
             (unsigned long)info.sample_rate, info.channels);
    while (remaining > 0) {
        size_t wanted = remaining < sizeof(audio_input) ? remaining : sizeof(audio_input);
        wanted -= wanted % input_frame_bytes;
        size_t received = wanted ? fread(audio_input, 1, wanted, file) : 0;
        if (received == 0) {
            if (remaining != 0) {
                result = ESP_FAIL;
                ESP_LOGE(TAG, "Unexpected end of %s", DOORBELL_WAV_PATH);
            }
            break;
        }

        int16_t *input = (int16_t *)audio_input;
        size_t output_bytes;
        if (info.channels == 1) {
            size_t samples = received / sizeof(int16_t);
            for (size_t index = 0; index < samples; ++index) {
                int16_t sample = input[index] >> 1;
                audio_output[index * 2] = sample;
                audio_output[index * 2 + 1] = sample;
            }
            output_bytes = received * 2;
        } else {
            size_t frames = received / (2 * sizeof(int16_t));
            for (size_t index = 0; index < frames; ++index) {
                int32_t mixed = (int32_t)input[index * 2] + input[index * 2 + 1];
                int16_t sample = (int16_t)(mixed / 2);
                audio_output[index * 2] = sample;
                audio_output[index * 2 + 1] = sample;
            }
            output_bytes = frames * 2 * sizeof(int16_t);
        }

        size_t written = 0;
        result = i2s_channel_write(channel, audio_output, output_bytes, &written, portMAX_DELAY);
        if (result != ESP_OK || written != output_bytes) {
            ESP_LOGE(TAG, "I2S write failed: %s", esp_err_to_name(result));
            if (result == ESP_OK) {
                result = ESP_FAIL;
            }
            break;
        }
        remaining -= received;
    }

    /* Let the final DMA samples leave the amplifier before shutting it down. */
    uint32_t drain_ms = (480U * 12U * 1000U) / info.sample_rate + 20U;
    vTaskDelay(pdMS_TO_TICKS(drain_ms));
    i2s_channel_disable(channel);
    i2s_del_channel(channel);
    fclose(file);
    return result;
}

static void doorbell_audio_task(void *arg)
{
    (void)arg;
    esp_err_t result = play_doorbell_wav();
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "Doorbell playback failed: %s", esp_err_to_name(result));
    }
    doorbell_audio_done = true;
    vTaskDelete(NULL);
}

static void start_doorbell_sequence(uint8_t event_id)
{
    if (doorbell_sequence_active) {
        return;
    }

    ESP_LOGI(TAG, "Starting media for BLE event 0x%02x", event_id);
    doorbell_sequence_active = true;
    knock_animation_done = false;
    doorbell_audio_done = false;
    /* Exactly two full passes, then LV_EVENT_READY marks it complete. */
    if (!set_character_animation(KNOCK_GIF_PATH, 2)) {
        /* Audio should still play even when the optional event GIF is absent. */
        knock_animation_done = true;
        (void)set_character_animation(IDLE_GIF_PATH, 0);
    }

    if (xTaskCreatePinnedToCore(doorbell_audio_task, "doorbell_audio", 4096,
                                NULL, 17, NULL, 0) != pdPASS) {
        ESP_LOGE(TAG, "Cannot create doorbell audio task");
        doorbell_audio_done = true;
    }
}

static void start_home_ui(void);

static void home_task(void *arg)
{
    (void)arg;
    /* The GIF decoder needs more stack than ESP-IDF's default main task. */
    start_home_ui();
    ESP_LOGI(TAG, "Media UI ready; idle animation is active");

    while (true) {
        uint8_t event_id;
        if (xQueueReceive(media_trigger_queue, &event_id, 0) == pdTRUE) {
            if (doorbell_sequence_active) {
                ESP_LOGW(TAG, "Ignoring BLE event 0x%02x while media is active", event_id);
            } else {
                start_doorbell_sequence(event_id);
            }
        }
        if (doorbell_sequence_active && knock_animation_done && doorbell_audio_done) {
            doorbell_sequence_active = false;
            knock_animation_done = false;
            (void)set_character_animation(IDLE_GIF_PATH, 0);
            ESP_LOGI(TAG, "Returned to idle animation");
        }

        uint32_t wait_ms = lv_timer_handler();
        if (wait_ms < 5) {
            wait_ms = 5;
        } else if (wait_ms > 20) {
            wait_ms = 20;
        }
        vTaskDelay(pdMS_TO_TICKS(wait_ms));
    }
}

static void start_home_ui(void)
{
    background_rgb565.header.magic = LV_IMAGE_HEADER_MAGIC;
    background_rgb565.header.cf = LV_COLOR_FORMAT_RGB565;
    background_rgb565.header.w = TFT_WIDTH;
    background_rgb565.header.h = TFT_HEIGHT;
    background_rgb565.header.stride = TFT_WIDTH * 2;
    background_rgb565.data_size = (uint32_t)(background_rgb565_end - background_rgb565_start);
    background_rgb565.data = background_rgb565_start;

    lv_init();
    lv_fs_posix_init();
    lv_display_t *display = lv_display_create(TFT_WIDTH, TFT_HEIGHT);
    lv_display_set_color_format(display, LV_COLOR_FORMAT_RGB565);
    lv_display_set_buffers(display, lvgl_draw_buf1, lvgl_draw_buf2, sizeof(lvgl_draw_buf1),
                           LV_DISPLAY_RENDER_MODE_PARTIAL);
    lv_display_set_flush_cb(display, display_flush_cb);
    create_home_ui(display);

    const esp_timer_create_args_t tick_args = {
        .callback = lvgl_tick_callback,
        .name = "lvgl_tick",
    };
    esp_timer_handle_t tick_timer;
    ESP_ERROR_CHECK(esp_timer_create(&tick_args, &tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(tick_timer, 2000));
}

esp_err_t doorbell_media_init(void)
{
    if (media_trigger_queue != NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    media_trigger_queue = xQueueCreate(1, sizeof(uint8_t));
    if (media_trigger_queue == NULL) {
        return ESP_ERR_NO_MEM;
    }

    gpio_config_t outputs = {
        .pin_bit_mask = (1ULL << TFT_RST_GPIO) | (1ULL << TFT_BL_GPIO),
        .mode = GPIO_MODE_OUTPUT,
    };
    esp_err_t result = gpio_config(&outputs);
    if (result != ESP_OK) {
        return result;
    }
    gpio_set_level(TFT_BL_GPIO, !TFT_BL_ON_LEVEL);

    spi_bus_config_t tft_bus = {
        .sclk_io_num = TFT_SCLK_GPIO,
        .mosi_io_num = TFT_MOSI_GPIO,
        .miso_io_num = -1,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = TFT_WIDTH * LVGL_DRAW_LINES * 2,
    };
    result = spi_bus_initialize(TFT_SPI_HOST, &tft_bus, SPI_DMA_CH_AUTO);
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "TFT SPI bus init failed: %s", esp_err_to_name(result));
        return result;
    }
    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = TFT_DC_GPIO,
        .cs_gpio_num = TFT_CS_GPIO,
        .pclk_hz = 40 * 1000 * 1000,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .spi_mode = 0,
        .trans_queue_depth = 1,
    };
    result = esp_lcd_new_panel_io_spi(TFT_SPI_HOST, &io_config, &lcd_io);
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "TFT panel I/O init failed: %s", esp_err_to_name(result));
        return result;
    }
    lcd_reset_and_init();
    gpio_set_level(TFT_BL_GPIO, TFT_BL_ON_LEVEL);

    sdmmc_card_t *card = NULL;
    result = mount_sdcard(&card);
    if (result == ESP_OK) {
        ESP_LOGI(TAG, "SD card mounted at %s", MOUNT_POINT);
        sdmmc_card_print_info(stdout, card);
        measure_sd_read_speed();
    } else {
        /* Keep the embedded background/UI alive so the fault is visible. */
        ESP_LOGE(TAG, "SD mount failed: %s", esp_err_to_name(result));
    }

    if (xTaskCreatePinnedToCore(home_task, "home_ui", 8192, NULL, 4, NULL, 1) != pdPASS) {
        ESP_LOGE(TAG, "Cannot create home UI task");
        return ESP_ERR_NO_MEM;
    }
    /* The queue can safely retain an event while the UI task finishes init. */
    media_initialized = true;

    return ESP_OK;
}

esp_err_t doorbell_media_trigger(uint8_t event_id)
{
    if (!media_initialized || media_trigger_queue == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    return xQueueOverwrite(media_trigger_queue, &event_id) == pdTRUE ? ESP_OK : ESP_FAIL;
}
