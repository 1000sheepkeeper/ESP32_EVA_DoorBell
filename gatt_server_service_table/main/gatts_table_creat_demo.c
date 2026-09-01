/*
 * SPDX-FileCopyrightText: 2021-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

/****************************************************************************
*
* BLE GATT Server - Smart Doorbell (Slave/Peripheral) with local audio
*
* Power-saving communication model (on-demand connection):
* - The host keeps scanning; while idle the slave does NOT advertise and
*   does NOT maintain a standing connection.
* - Pressing the doorbell button (GPIO16, active high) starts advertising.
*   The host discovers the device, connects and enables notifications, and
*   the slave immediately delivers the deferred doorbell notification.
*   After the notification is acknowledged the slave closes the link and
*   goes back to idle until the next press.
* - The doorbell press also plays the embedded doorbell.wav through I2S
*   (MAX98357A) locally, with a +12 dB software boost.
* - GPIO17 volume up / GPIO15 volume down: adjust the I2S attenuation.
*
****************************************************************************/


#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "driver/gpio.h"
#include "esp_timer.h"

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "gatts_table_creat_demo.h"
#include "esp_gatt_common_api.h"
#include "audio_player.h"

#define GATTS_TABLE_TAG "DOORBELL_SLAVE"

#define PROFILE_NUM                 1
#define PROFILE_APP_IDX             0
#define ESP_APP_ID                  0x55
#define SAMPLE_DEVICE_NAME          "ESP_GATTS_DEMO"
#define SVC_INST_ID                 0

#define GATTS_DEMO_CHAR_VAL_LEN_MAX 500
#define PREPARE_BUF_MAX_SIZE        1024
#define CHAR_DECLARATION_SIZE       (sizeof(uint8_t))

#define ADV_CONFIG_FLAG             (1 << 0)
#define SCAN_RSP_CONFIG_FLAG        (1 << 1)

/* GPIO configuration - buttons are ACTIVE HIGH, internal pull-down enabled.
 * Doorbell button is wired to GPIO16 (verified on the actual doorbell board).
 * GPIO17 / GPIO15 are the volume up / down buttons (if wired). */
#define GPIO_BTN_DOORBELL        GPIO_NUM_16   /* doorbell: BLE notify + local audio */
#define GPIO_BTN_VOL_UP          GPIO_NUM_17   /* volume up */
#define GPIO_BTN_VOL_DOWN        GPIO_NUM_15   /* volume down */
#define GPIO_BUTTON_ACTIVE_LEVEL 1             /* Active high */

#define BUTTON_TASK_PRIO         10
#define BUTTON_TASK_STACK        3072
#define BUTTON_POLL_MS           20
#define BUTTON_DEBOUNCE_MS       30

/* After a doorbell press, wait this long for the host to connect before
 * stopping advertising and returning to idle. */
#define ADV_CONNECT_WINDOW_MS    (60 * 1000)
#define ADV_RETRY_DELAY_US       (500 * 1000)
#define ADV_STOP_RETRY_MAX       3

/* Connection ID zero is valid (and is normally assigned to the first link).
 * Use an out-of-range value when no link is saved instead of using zero as a
 * sentinel. */
#define INVALID_CONN_ID         UINT16_MAX
#define NOTIFY_CLOSE_DELAY_US   (200 * 1000)
#define NOTIFY_CLOSE_RETRY_MAX  3

/* ESP32 BLE maximum transmit-power level: +9 dBm. */
#define BLE_TX_POWER_LEVEL          ESP_PWR_LVL_P9

#define ADV_CONFIG_ALL            (ADV_CONFIG_FLAG | SCAN_RSP_CONFIG_FLAG)

static uint8_t adv_config_pending = 0;
static uint8_t adv_configured = 0;

uint16_t heart_rate_handle_table[HRS_IDX_NB];

typedef struct {
    uint8_t                 *prepare_buf;
    int                     prepare_len;
} prepare_type_env_t;

static prepare_type_env_t prepare_write_env;

/* State flags */
static volatile bool notify_enabled = false;
static volatile bool notify_need_confirm = false;
static volatile bool notification_sent = false;
static volatile bool pending_notify = false;
static volatile bool ble_connected = false;
static volatile uint16_t saved_conn_id = INVALID_CONN_ID;
static volatile esp_gatt_if_t saved_gatts_if = ESP_GATT_IF_NONE;
static volatile uint32_t connection_generation = 0;
static esp_bd_addr_t saved_remote_bda = {0};

/* Closing a link from the GATT callback after a blocking delay stalls the
 * Bluedroid callback task.  A one-shot timer performs the close later from
 * the esp_timer task. */
static esp_timer_handle_t notify_close_timer = NULL;
static uint16_t notify_close_conn_id = INVALID_CONN_ID;
static esp_gatt_if_t notify_close_gatts_if = ESP_GATT_IF_NONE;
static uint32_t notify_close_generation = 0;
static volatile bool notify_close_pending = false;
static volatile bool notify_close_requested = false;
static uint8_t notify_close_retries = 0;

/* Advertising-on-demand state (idle -> press -> advertise -> connect -> notify -> idle) */
typedef enum {
    ADV_IDLE = 0,
    ADV_STARTING,
    ADV_ACTIVE,
    ADV_STOPPING,
} advertising_state_t;

static volatile bool adv_wanted = false;         /* advertising requested (doorbell pressed) */
static volatile advertising_state_t adv_state = ADV_IDLE;
static uint8_t adv_stop_retries = 0;
static esp_timer_handle_t adv_timeout_timer = NULL;
static esp_timer_handle_t adv_retry_timer = NULL;

#define CONFIG_SET_RAW_ADV_DATA
#ifdef CONFIG_SET_RAW_ADV_DATA
static uint8_t raw_adv_data[] = {
    /* Flags */
    0x02, ESP_BLE_AD_TYPE_FLAG, 0x06,
    /* TX Power Level */
    0x02, ESP_BLE_AD_TYPE_TX_PWR, 0x09,
    /* Complete 16-bit Service UUIDs */
    0x03, ESP_BLE_AD_TYPE_16SRV_CMPL, 0xFF, 0x00,
    /* Complete Local Name */
    0x0F, ESP_BLE_AD_TYPE_NAME_CMPL,
    'E', 'S', 'P', '_', 'G', 'A', 'T', 'T', 'S', '_', 'D', 'E', 'M', 'O'
};

static uint8_t raw_scan_rsp_data[] = {
    /* Flags */
    0x02, ESP_BLE_AD_TYPE_FLAG, 0x06,
    /* TX Power Level */
    0x02, ESP_BLE_AD_TYPE_TX_PWR, 0x09,
    /* Complete 16-bit Service UUIDs */
    0x03, ESP_BLE_AD_TYPE_16SRV_CMPL, 0xFF, 0x00
};
#endif

static esp_ble_adv_params_t adv_params = {
    .adv_int_min         = ESP_BLE_GAP_ADV_ITVL_MS(20),
    .adv_int_max         = ESP_BLE_GAP_ADV_ITVL_MS(40),
    .adv_type            = ADV_TYPE_IND,
    .own_addr_type       = BLE_ADDR_TYPE_PUBLIC,
    .channel_map         = ADV_CHNL_ALL,
    .adv_filter_policy   = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

struct gatts_profile_inst {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
};

static void gatts_profile_event_handler(esp_gatts_cb_event_t event,
					esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

static struct gatts_profile_inst heart_rate_profile_tab[PROFILE_NUM] = {
    [PROFILE_APP_IDX] = {
        .gatts_cb = gatts_profile_event_handler,
        .gatts_if = ESP_GATT_IF_NONE,
    },
};

/* Service UUIDs */
static const uint16_t GATTS_SERVICE_UUID_TEST      = 0x00FF;
static const uint16_t GATTS_CHAR_UUID_TEST_A       = 0xFF01;
static const uint16_t GATTS_CHAR_UUID_TEST_B       = 0xFF02;
static const uint16_t GATTS_CHAR_UUID_TEST_C       = 0xFF03;

static const uint16_t primary_service_uuid         = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid   = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
static const uint8_t char_prop_read                =  ESP_GATT_CHAR_PROP_BIT_READ;
static const uint8_t char_prop_write               = ESP_GATT_CHAR_PROP_BIT_WRITE;
static const uint8_t char_prop_read_write_notify   = ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint8_t heart_measurement_ccc[2]      = {0x00, 0x00};
static const uint8_t char_value[4]                 = {0x11, 0x22, 0x33, 0x44};

/* Full Database Description */
static const esp_gatts_attr_db_t gatt_db[HRS_IDX_NB] =
{
    [IDX_SVC]        =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ,
      sizeof(uint16_t), sizeof(GATTS_SERVICE_UUID_TEST), (uint8_t *)&GATTS_SERVICE_UUID_TEST}},

    [IDX_CHAR_A]     =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},

    [IDX_CHAR_VAL_A] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_TEST_A, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value), (uint8_t *)char_value}},

    [IDX_CHAR_CFG_A]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      sizeof(uint16_t), sizeof(heart_measurement_ccc), (uint8_t *)heart_measurement_ccc}},

    [IDX_CHAR_B]      =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read}},

    [IDX_CHAR_VAL_B]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_TEST_B, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value), (uint8_t *)char_value}},

    [IDX_CHAR_C]      =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_write}},

    [IDX_CHAR_VAL_C]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_TEST_C, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value), (uint8_t *)char_value}},
};

/* Advertising-on-demand helpers (defined below) */
static void start_doorbell_advertising(void);
static void stop_doorbell_advertising(void);
static void try_start_doorbell_advertising(void);
static void schedule_advertising_retry(void);
static void request_advertising_stop(void);

#ifdef CONFIG_SET_RAW_ADV_DATA
static void configure_raw_advertising_data(uint8_t config_mask)
{
    if ((config_mask & ADV_CONFIG_FLAG) != 0 &&
            (adv_config_pending & ADV_CONFIG_FLAG) == 0 &&
            (adv_configured & ADV_CONFIG_FLAG) == 0) {
        esp_err_t ret = esp_ble_gap_config_adv_data_raw(
            raw_adv_data, sizeof(raw_adv_data));
        if (ret == ESP_OK) {
            adv_config_pending |= ADV_CONFIG_FLAG;
        } else {
            ESP_LOGW(GATTS_TABLE_TAG, "config raw advertising data failed: %s",
                     esp_err_to_name(ret));
        }
    }

    if ((config_mask & SCAN_RSP_CONFIG_FLAG) != 0 &&
            (adv_config_pending & SCAN_RSP_CONFIG_FLAG) == 0 &&
            (adv_configured & SCAN_RSP_CONFIG_FLAG) == 0) {
        esp_err_t ret = esp_ble_gap_config_scan_rsp_data_raw(
            raw_scan_rsp_data, sizeof(raw_scan_rsp_data));
        if (ret == ESP_OK) {
            adv_config_pending |= SCAN_RSP_CONFIG_FLAG;
        } else {
            ESP_LOGW(GATTS_TABLE_TAG, "config raw scan response failed: %s",
                     esp_err_to_name(ret));
        }
    }
}

static bool advertising_config_ready(void)
{
    return adv_config_pending == 0 && adv_configured == ADV_CONFIG_ALL;
}
#else
static bool advertising_config_ready(void)
{
    return true;
}
#endif

static void advertising_retry_timer_cb(void *arg)
{
    (void)arg;

    if (adv_state == ADV_STOPPING) {
        request_advertising_stop();
        return;
    }

    if (!adv_wanted || ble_connected || adv_state != ADV_IDLE) {
        return;
    }

#ifdef CONFIG_SET_RAW_ADV_DATA
    if (!advertising_config_ready()) {
        configure_raw_advertising_data(ADV_CONFIG_ALL);
        schedule_advertising_retry();
        return;
    }
#endif
    try_start_doorbell_advertising();
}

static void schedule_advertising_retry(void)
{
    if (adv_retry_timer == NULL ||
            (!adv_wanted && adv_state != ADV_STOPPING)) {
        return;
    }

    (void)esp_timer_stop(adv_retry_timer);
    esp_err_t ret = esp_timer_start_once(adv_retry_timer, ADV_RETRY_DELAY_US);
    if (ret != ESP_OK) {
        ESP_LOGW(GATTS_TABLE_TAG, "Failed to schedule advertising retry: %s",
                 esp_err_to_name(ret));
    }
}

static void notify_close_timer_cb(void *arg)
{
    (void)arg;

    if (!notify_close_pending) {
        return;
    }

    const uint16_t conn_id = notify_close_conn_id;
    const esp_gatt_if_t gatts_if = notify_close_gatts_if;
    const uint32_t generation = notify_close_generation;
    notify_close_pending = false;

    /* A disconnect can race with the timer callback.  Never close a newer
     * connection using the old callback's connection ID. */
    if (!ble_connected || generation != connection_generation ||
            conn_id == INVALID_CONN_ID ||
            saved_conn_id != conn_id || saved_gatts_if != gatts_if) {
        ESP_LOGD(GATTS_TABLE_TAG, "Skipping stale notification close request");
        return;
    }

    esp_err_t ret = esp_ble_gatts_close(gatts_if, conn_id);
    if (ret != ESP_OK) {
        ESP_LOGW(GATTS_TABLE_TAG, "Failed to close connection %u: %s",
                 conn_id, esp_err_to_name(ret));
        if (notify_close_retries++ < NOTIFY_CLOSE_RETRY_MAX &&
                ble_connected && generation == connection_generation &&
                saved_conn_id == conn_id && saved_gatts_if == gatts_if) {
            notify_close_pending = true;
            esp_err_t retry_ret = esp_timer_start_once(
                notify_close_timer, NOTIFY_CLOSE_DELAY_US);
            if (retry_ret != ESP_OK) {
                notify_close_pending = false;
                ESP_LOGW(GATTS_TABLE_TAG,
                         "Failed to retry notification close: %s",
                         esp_err_to_name(retry_ret));
            }
        }
    } else {
        notify_close_requested = true;
        ESP_LOGI(GATTS_TABLE_TAG, "Notification delivered; closing connection %u",
                 conn_id);
    }
}

static void schedule_notification_close(esp_gatt_if_t gatts_if, uint16_t conn_id)
{
    if (notify_close_timer == NULL) {
        ESP_LOGW(GATTS_TABLE_TAG, "Notification close timer is not ready");
        return;
    }

    notify_close_conn_id = conn_id;
    notify_close_gatts_if = gatts_if;
    notify_close_generation = connection_generation;
    notify_close_pending = true;
    notify_close_requested = false;
    notify_close_retries = 0;

    /* A second successful confirmation supersedes an older close request. */
    (void)esp_timer_stop(notify_close_timer);
    esp_err_t ret = esp_timer_start_once(notify_close_timer, NOTIFY_CLOSE_DELAY_US);
    if (ret != ESP_OK) {
        notify_close_pending = false;
        ESP_LOGW(GATTS_TABLE_TAG, "Failed to schedule notification close: %s",
                 esp_err_to_name(ret));
    }
}

/* Send the doorbell-trigger notification to the connected host (if any).
 * Payload: [0] = event type (1 = doorbell press), [1] = button level.
 * If the host is not connected/subscribed yet, the notification is deferred
 * and advertising is started so the (constantly scanning) host can connect. */
static void send_doorbell_notify(void)
{
    if (notification_sent) {
        /* This implementation intentionally coalesces a second press while
         * the current event is being delivered or its link is closing. */
        pending_notify = true;
        ESP_LOGW(GATTS_TABLE_TAG,
                 "Notification already in flight; retaining one pending event");
        return;
    }

    if (!notify_enabled || !ble_connected || saved_conn_id == INVALID_CONN_ID) {
        pending_notify = true;
        ESP_LOGW(GATTS_TABLE_TAG,
                 "Host not connected/subscribed - notification deferred");
        if (!ble_connected) {
            start_doorbell_advertising();
        }
        return;
    }

    uint8_t notify_data[4] = {0x01, GPIO_BUTTON_ACTIVE_LEVEL, 0x00, 0x00};
    esp_err_t ret = esp_ble_gatts_send_indicate(saved_gatts_if, saved_conn_id,
                                                heart_rate_handle_table[IDX_CHAR_VAL_A],
                                                sizeof(notify_data), notify_data,
                                                notify_need_confirm);
    if (ret == ESP_OK) {
        notification_sent = true;
        pending_notify = false;
        ESP_LOGI(GATTS_TABLE_TAG, "Doorbell notification sent to host");
        if (!notify_need_confirm) {
            /* Bluedroid does not require a peer confirmation for a
             * notification, so do not depend on CONF_EVT to close the link. */
            schedule_notification_close(saved_gatts_if, saved_conn_id);
        }
    } else {
        pending_notify = true;
        ESP_LOGE(GATTS_TABLE_TAG, "Failed to send notification: %s", esp_err_to_name(ret));
        if (ble_connected && saved_conn_id != INVALID_CONN_ID) {
            /* Drop the unusable link asynchronously; DISCONNECT_EVT will
             * restart advertising for the retained event. */
            schedule_notification_close(saved_gatts_if, saved_conn_id);
        }
    }
}

/* Advertising timeout: no host connected within the window -> back to idle */
static void adv_timeout_cb(void *arg)
{
    ESP_LOGW(GATTS_TABLE_TAG, "No host connected within %d s, advertising stopped (press button to ring again)",
             ADV_CONNECT_WINDOW_MS / 1000);
    adv_wanted = false;
    pending_notify = false;
    if (!ble_connected) {
        stop_doorbell_advertising();
    }
}

static void try_start_doorbell_advertising(void)
{
    if (!adv_wanted || ble_connected || adv_state != ADV_IDLE) {
        return;
    }

    if (!advertising_config_ready()) {
        ESP_LOGI(GATTS_TABLE_TAG, "Advertising data is still being configured");
        schedule_advertising_retry();
        return;
    }

    adv_state = ADV_STARTING;
    esp_err_t ret = esp_ble_gap_start_advertising(&adv_params);
    if (ret != ESP_OK) {
        adv_state = ADV_IDLE;
        ESP_LOGW(GATTS_TABLE_TAG, "start advertising failed: %s",
                 esp_err_to_name(ret));
        schedule_advertising_retry();
    }
}

/* Start advertising for the on-demand connection (called on doorbell press). */
static void start_doorbell_advertising(void)
{
    adv_wanted = true;
    if (adv_timeout_timer) {
        (void)esp_timer_stop(adv_timeout_timer);
        esp_err_t timer_ret = esp_timer_start_once(
            adv_timeout_timer, ADV_CONNECT_WINDOW_MS * 1000);
        if (timer_ret != ESP_OK) {
            ESP_LOGW(GATTS_TABLE_TAG, "Failed to start advertising timeout: %s",
                     esp_err_to_name(timer_ret));
        }
    }
    try_start_doorbell_advertising();
}

static void request_advertising_stop(void)
{
    if (adv_state != ADV_ACTIVE && adv_state != ADV_STOPPING) {
        return;
    }

    if (adv_state == ADV_ACTIVE) {
        adv_state = ADV_STOPPING;
        adv_stop_retries = 0;
    } else if (adv_stop_retries++ >= ADV_STOP_RETRY_MAX) {
        /* A missing completion event must not block future button presses.
         * Starting again below is still safe if the controller did stop; if
         * it did not, the synchronous start call will fail and be retried. */
        ESP_LOGW(GATTS_TABLE_TAG,
                 "Advertising stop did not complete; returning to recoverable idle state");
        adv_state = ADV_IDLE;
        adv_stop_retries = 0;
        try_start_doorbell_advertising();
        return;
    }

    esp_err_t ret = esp_ble_gap_stop_advertising();
    if (ret != ESP_OK) {
        ESP_LOGW(GATTS_TABLE_TAG, "stop advertising request failed: %s",
                 esp_err_to_name(ret));
    }
    /* Keep STOPPING until the completion event.  The timer is also armed
     * after a successful request so a lost completion event is recoverable. */
    schedule_advertising_retry();
}

/* Stop advertising and return to idle (called on disconnect). */
static void stop_doorbell_advertising(void)
{
    adv_wanted = false;
    if (adv_timeout_timer) {
        (void)esp_timer_stop(adv_timeout_timer);
    }
    if (adv_retry_timer) {
        (void)esp_timer_stop(adv_retry_timer);
    }
    if (adv_state == ADV_ACTIVE) {
        request_advertising_stop();
    }
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    #ifdef CONFIG_SET_RAW_ADV_DATA
        case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
            if (param->adv_data_raw_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGW(GATTS_TABLE_TAG, "Raw advertising data setup failed");
                adv_config_pending &= ~ADV_CONFIG_FLAG;
                adv_configured &= ~ADV_CONFIG_FLAG;
                configure_raw_advertising_data(ADV_CONFIG_FLAG);
                schedule_advertising_retry();
            } else {
                adv_config_pending &= ~ADV_CONFIG_FLAG;
                adv_configured |= ADV_CONFIG_FLAG;
            }
            if (adv_wanted && advertising_config_ready()) {
                try_start_doorbell_advertising();
            }
            break;
        case ESP_GAP_BLE_SCAN_RSP_DATA_RAW_SET_COMPLETE_EVT:
            if (param->scan_rsp_data_raw_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGW(GATTS_TABLE_TAG, "Raw scan response setup failed");
                adv_config_pending &= ~SCAN_RSP_CONFIG_FLAG;
                adv_configured &= ~SCAN_RSP_CONFIG_FLAG;
                configure_raw_advertising_data(SCAN_RSP_CONFIG_FLAG);
                schedule_advertising_retry();
            } else {
                adv_config_pending &= ~SCAN_RSP_CONFIG_FLAG;
                adv_configured |= SCAN_RSP_CONFIG_FLAG;
            }
            if (adv_wanted && advertising_config_ready()) {
                try_start_doorbell_advertising();
            }
            break;
    #endif
        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            if (adv_state != ADV_STARTING) {
                ESP_LOGW(GATTS_TABLE_TAG, "Ignoring stale advertising-start completion");
                break;
            }
            if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(GATTS_TABLE_TAG, "advertising start failed, status %d",
                         param->adv_start_cmpl.status);
                adv_state = ADV_IDLE;
                schedule_advertising_retry();
            }else{
                ESP_LOGI(GATTS_TABLE_TAG, "advertising start successfully");
                adv_state = ADV_ACTIVE;
                if (!adv_wanted || ble_connected) {
                    request_advertising_stop();
                }
            }
            break;
        case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
            if (adv_state != ADV_STOPPING) {
                ESP_LOGW(GATTS_TABLE_TAG, "Ignoring stale advertising-stop completion");
                break;
            }
            if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(GATTS_TABLE_TAG, "Advertising stop failed, status %d",
                         param->adv_stop_cmpl.status);
                schedule_advertising_retry();
            }
            else {
                ESP_LOGI(GATTS_TABLE_TAG, "Stop adv successfully");
                adv_state = ADV_IDLE;
                adv_stop_retries = 0;
                if (adv_retry_timer) {
                    (void)esp_timer_stop(adv_retry_timer);
                }
                if (adv_wanted && !ble_connected) {
                    try_start_doorbell_advertising();
                }
            }
            break;
        case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "update connection params status = %d, conn_int = %d, latency = %d, timeout = %d",
                  param->update_conn_params.status,
                  param->update_conn_params.conn_int,
                  param->update_conn_params.latency,
                  param->update_conn_params.timeout);
            break;
        default:
            break;
    }
}

void example_prepare_write_event_env(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param)
{
    ESP_LOGI(GATTS_TABLE_TAG, "prepare write, handle = %d, value len = %d", param->write.handle, param->write.len);
    esp_gatt_status_t status = ESP_GATT_OK;
    if (param->write.offset > PREPARE_BUF_MAX_SIZE) {
        status = ESP_GATT_INVALID_OFFSET;
    } else if ((param->write.offset + param->write.len) > PREPARE_BUF_MAX_SIZE) {
        status = ESP_GATT_INVALID_ATTR_LEN;
    }
    if (status == ESP_GATT_OK && prepare_write_env->prepare_buf == NULL) {
        prepare_write_env->prepare_buf = (uint8_t *)malloc(PREPARE_BUF_MAX_SIZE * sizeof(uint8_t));
        prepare_write_env->prepare_len = 0;
        if (prepare_write_env->prepare_buf == NULL) {
            ESP_LOGE(GATTS_TABLE_TAG, "%s, Gatt_server prep no mem", __func__);
            status = ESP_GATT_NO_RESOURCES;
        }
    }

    if (param->write.need_rsp){
        esp_gatt_rsp_t *gatt_rsp = (esp_gatt_rsp_t *)malloc(sizeof(esp_gatt_rsp_t));
        if (gatt_rsp != NULL){
            gatt_rsp->attr_value.len = param->write.len;
            gatt_rsp->attr_value.handle = param->write.handle;
            gatt_rsp->attr_value.offset = param->write.offset;
            gatt_rsp->attr_value.auth_req = ESP_GATT_AUTH_REQ_NONE;
            memcpy(gatt_rsp->attr_value.value, param->write.value, param->write.len);
            esp_err_t response_err = esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, gatt_rsp);
            if (response_err != ESP_OK) {
               ESP_LOGE(GATTS_TABLE_TAG, "Send response error");
            }
            free(gatt_rsp);
        }else{
            ESP_LOGE(GATTS_TABLE_TAG, "%s, malloc failed", __func__);
            status = ESP_GATT_NO_RESOURCES;
        }
    }
    if (status != ESP_GATT_OK){
        return;
    }
    memcpy(prepare_write_env->prepare_buf + param->write.offset,
           param->write.value,
           param->write.len);
    prepare_write_env->prepare_len += param->write.len;

}

void example_exec_write_event_env(prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param){
    if (param->exec_write.exec_write_flag == ESP_GATT_PREP_WRITE_EXEC && prepare_write_env->prepare_buf){
        ESP_LOG_BUFFER_HEX(GATTS_TABLE_TAG, prepare_write_env->prepare_buf, prepare_write_env->prepare_len);
    }else{
        ESP_LOGI(GATTS_TABLE_TAG,"ESP_GATT_PREP_WRITE_CANCEL");
    }
    if (prepare_write_env->prepare_buf) {
        free(prepare_write_env->prepare_buf);
        prepare_write_env->prepare_buf = NULL;
    }
    prepare_write_env->prepare_len = 0;
}

static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    switch (event) {
        case ESP_GATTS_REG_EVT:{
            esp_err_t set_dev_name_ret = esp_ble_gap_set_device_name(SAMPLE_DEVICE_NAME);
            if (set_dev_name_ret){
                ESP_LOGE(GATTS_TABLE_TAG, "set device name failed, error code = %x", set_dev_name_ret);
            }
    #ifdef CONFIG_SET_RAW_ADV_DATA
            adv_config_pending = 0;
            adv_configured = 0;
            configure_raw_advertising_data(ADV_CONFIG_ALL);
    #endif
            esp_err_t create_attr_ret = esp_ble_gatts_create_attr_tab(gatt_db, gatts_if, HRS_IDX_NB, SVC_INST_ID);
            if (create_attr_ret){
                ESP_LOGE(GATTS_TABLE_TAG, "create attr table failed, error code = %x", create_attr_ret);
            }
        }
       	    break;
        case ESP_GATTS_READ_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_READ_EVT");
       	    break;
        case ESP_GATTS_WRITE_EVT:
            if (!param->write.is_prep){
                ESP_LOGI(GATTS_TABLE_TAG, "GATT_WRITE_EVT, handle = %d, value len = %d, value :", param->write.handle, param->write.len);
                ESP_LOG_BUFFER_HEX(GATTS_TABLE_TAG, param->write.value, param->write.len);

                bool deliver_pending = false;
                esp_gatt_status_t response_status = ESP_GATT_OK;
                bool current_connection = ble_connected &&
                    saved_conn_id == param->write.conn_id &&
                    saved_gatts_if == gatts_if;
                if (heart_rate_handle_table[IDX_CHAR_CFG_A] == param->write.handle &&
                        param->write.len == 2 && current_connection){
                    uint16_t descr_value = param->write.value[1]<<8 | param->write.value[0];
                    if (descr_value == 0x0001){
                        ESP_LOGI(GATTS_TABLE_TAG, "notify enable");
                        notify_enabled = true;
                        notify_need_confirm = false;
                        if (pending_notify) {
                            deliver_pending = true;
                        }
                    }else if (descr_value == 0x0002){
                        ESP_LOGI(GATTS_TABLE_TAG, "indicate enable");
                        notify_enabled = true;
                        notify_need_confirm = true;
                        if (pending_notify) {
                            deliver_pending = true;
                        }
                    }
                    else if (descr_value == 0x0000){
                        ESP_LOGI(GATTS_TABLE_TAG, "notify/indicate disable ");
                        notify_enabled = false;
                        notify_need_confirm = false;
                    }else{
                        ESP_LOGE(GATTS_TABLE_TAG, "unknown descr value");
                        ESP_LOG_BUFFER_HEX(GATTS_TABLE_TAG, param->write.value, param->write.len);
                        response_status = ESP_GATT_INVALID_ATTR_LEN;
                    }
                } else if (heart_rate_handle_table[IDX_CHAR_CFG_A] == param->write.handle &&
                           param->write.len != 2) {
                    response_status = ESP_GATT_INVALID_ATTR_LEN;
                }

                /* Respond to the CCCD write FIRST, then deliver the deferred
                 * notification: sending the notification before the ATT write
                 * response can be dropped by the stack. */
                if (param->write.need_rsp){
                    esp_err_t response_ret = esp_ble_gatts_send_response(
                        gatts_if, param->write.conn_id, param->write.trans_id,
                        response_status, NULL);
                    if (response_ret != ESP_OK) {
                        ESP_LOGW(GATTS_TABLE_TAG,
                                 "CCCD response failed: %s",
                                 esp_err_to_name(response_ret));
                    }
                }
                if (deliver_pending) {
                    ESP_LOGI(GATTS_TABLE_TAG, "Delivering deferred doorbell notification");
                    send_doorbell_notify();
                }
            }else{
                example_prepare_write_event_env(gatts_if, &prepare_write_env, param);
            }
      	    break;
        case ESP_GATTS_EXEC_WRITE_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_EXEC_WRITE_EVT");
            example_exec_write_event_env(&prepare_write_env, param);
            break;
        case ESP_GATTS_MTU_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_MTU_EVT, MTU %d", param->mtu.mtu);
            break;
        case ESP_GATTS_CONF_EVT:
            ESP_LOGI(GATTS_TABLE_TAG,
                     "ESP_GATTS_CONF_EVT, status = %d, conn_id %u, attr_handle %u",
                     param->conf.status, param->conf.conn_id, param->conf.handle);
            /* For a notification (need_confirm=false), this callback reports
             * the stack's transmit result; for an indication it follows the
             * peer confirmation.  In either case, do not block this callback
             * task while waiting before closing the short-lived link. */
            if (notification_sent && ble_connected &&
                    gatts_if == saved_gatts_if &&
                    param->conf.conn_id == saved_conn_id &&
                    param->conf.handle == heart_rate_handle_table[IDX_CHAR_VAL_A]) {
                if (param->conf.status == ESP_GATT_OK) {
                    if (notify_need_confirm ||
                            (!notify_close_pending && !notify_close_requested)) {
                        schedule_notification_close(gatts_if, param->conf.conn_id);
                    }
                } else {
                    notification_sent = false;
                    pending_notify = true;
                    ESP_LOGW(GATTS_TABLE_TAG,
                             "Notification delivery failed, status %d; retaining pending event",
                             param->conf.status);
                    schedule_notification_close(gatts_if, param->conf.conn_id);
                }
            }
            break;
        case ESP_GATTS_START_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "SERVICE_START_EVT, status %d, service_handle %d", param->start.status, param->start.service_handle);
            break;
        case ESP_GATTS_CONNECT_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_CONNECT_EVT, conn_id = %d", param->connect.conn_id);
            ESP_LOG_BUFFER_HEX(GATTS_TABLE_TAG, param->connect.remote_bda, 6);
            connection_generation++;
            saved_conn_id = param->connect.conn_id;
            saved_gatts_if = gatts_if;
            memcpy(saved_remote_bda, param->connect.remote_bda,
                   sizeof(saved_remote_bda));
            ble_connected = true;
            notify_enabled = false;
            notify_need_confirm = false;
            notification_sent = false;
            notify_close_pending = false;
            notify_close_requested = false;
            if (notify_close_timer) {
                (void)esp_timer_stop(notify_close_timer);
            }
            adv_wanted = false;                 /* the connection consumed advertising */
            if (adv_timeout_timer) {
                (void)esp_timer_stop(adv_timeout_timer);
            }
            if (adv_retry_timer) {
                (void)esp_timer_stop(adv_retry_timer);
            }
            if (adv_state == ADV_ACTIVE || adv_state == ADV_STOPPING) {
                adv_state = ADV_IDLE;
            }
            esp_ble_conn_update_params_t conn_params = {0};
            memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
            conn_params.latency = 0;
            conn_params.max_int = 0x20;    // max_int = 0x20*1.25ms = 40ms
            conn_params.min_int = 0x10;    // min_int = 0x10*1.25ms = 20ms
            conn_params.timeout = 400;    // timeout = 400*10ms = 4000ms
            esp_err_t conn_params_ret = esp_ble_gap_update_conn_params(&conn_params);
            if (conn_params_ret != ESP_OK) {
                ESP_LOGW(GATTS_TABLE_TAG, "Connection parameter update failed: %s",
                         esp_err_to_name(conn_params_ret));
            }
            break;
        case ESP_GATTS_DISCONNECT_EVT:
            if (!ble_connected ||
                    saved_conn_id != param->disconnect.conn_id ||
                    memcmp(saved_remote_bda, param->disconnect.remote_bda,
                           sizeof(saved_remote_bda)) != 0) {
                ESP_LOGW(GATTS_TABLE_TAG, "Ignoring stale disconnect event, conn_id %u",
                         param->disconnect.conn_id);
                break;
            }
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_DISCONNECT_EVT, reason = 0x%x", param->disconnect.reason);
            bool retry_pending = pending_notify;
            connection_generation++;
            ble_connected = false;
            notify_enabled = false;
            notify_need_confirm = false;
            notification_sent = false;
            notify_close_pending = false;
            notify_close_requested = false;
            if (notify_close_timer) {
                (void)esp_timer_stop(notify_close_timer);
            }
            saved_conn_id = INVALID_CONN_ID;
            saved_gatts_if = ESP_GATT_IF_NONE;
            memset(saved_remote_bda, 0, sizeof(saved_remote_bda));
            /* Retry a retained event after a failed delivery or a second press. */
            stop_doorbell_advertising();
            if (retry_pending) {
                start_doorbell_advertising();
            }
            break;
        case ESP_GATTS_CREAT_ATTR_TAB_EVT:{
            if (param->add_attr_tab.status != ESP_GATT_OK){
                ESP_LOGE(GATTS_TABLE_TAG, "create attribute table failed, error code=0x%x", param->add_attr_tab.status);
            }
            else if (param->add_attr_tab.num_handle != HRS_IDX_NB){
                ESP_LOGE(GATTS_TABLE_TAG, "create attribute table abnormally, num_handle (%d) \
                        doesn't equal to HRS_IDX_NB(%d)", param->add_attr_tab.num_handle, HRS_IDX_NB);
            }
            else {
                ESP_LOGI(GATTS_TABLE_TAG, "create attribute table successfully, the number handle = %d",param->add_attr_tab.num_handle);
                memcpy(heart_rate_handle_table, param->add_attr_tab.handles, sizeof(heart_rate_handle_table));
                esp_err_t start_ret = esp_ble_gatts_start_service(heart_rate_handle_table[IDX_SVC]);
                if (start_ret != ESP_OK) {
                    ESP_LOGE(GATTS_TABLE_TAG, "start service failed: %s",
                             esp_err_to_name(start_ret));
                }
            }
            break;
        }
        case ESP_GATTS_STOP_EVT:
        case ESP_GATTS_OPEN_EVT:
        case ESP_GATTS_CANCEL_OPEN_EVT:
        case ESP_GATTS_CLOSE_EVT:
        case ESP_GATTS_LISTEN_EVT:
        case ESP_GATTS_CONGEST_EVT:
        case ESP_GATTS_UNREG_EVT:
        case ESP_GATTS_DELETE_EVT:
        default:
            break;
    }
}


static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    if (event == ESP_GATTS_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            heart_rate_profile_tab[PROFILE_APP_IDX].gatts_if = gatts_if;
        } else {
            ESP_LOGE(GATTS_TABLE_TAG, "reg app failed, app_id %04x, status %d",
                    param->reg.app_id,
                    param->reg.status);
            return;
        }
    }
    do {
        int idx;
        for (idx = 0; idx < PROFILE_NUM; idx++) {
            if (gatts_if == ESP_GATT_IF_NONE || gatts_if == heart_rate_profile_tab[idx].gatts_if) {
                if (heart_rate_profile_tab[idx].gatts_cb) {
                    heart_rate_profile_tab[idx].gatts_cb(event, gatts_if, param);
                }
            }
        }
    } while (0);
}

/* Initialize GPIO for the three buttons (active high, internal pull-down) */
static void init_button_gpio(void)
{
    gpio_config_t io_conf = {
        .intr_type    = GPIO_INTR_DISABLE,
        .mode         = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << GPIO_BTN_DOORBELL) |
                        (1ULL << GPIO_BTN_VOL_UP) |
                        (1ULL << GPIO_BTN_VOL_DOWN),
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
    };
    gpio_config(&io_conf);
    ESP_LOGI(GATTS_TABLE_TAG, "Buttons initialized: doorbell=GPIO%d, vol_up=GPIO%d, vol_down=GPIO%d (active high, pull-down)",
             GPIO_BTN_DOORBELL, GPIO_BTN_VOL_UP, GPIO_BTN_VOL_DOWN);
}

/* Button monitoring task: doorbell -> BLE notify + local audio,
 * two other buttons -> volume up/down. Rising-edge + debounce. */
static void button_task(void *arg)
{
    int last_doorbell = gpio_get_level(GPIO_BTN_DOORBELL);
    int last_vol_up   = gpio_get_level(GPIO_BTN_VOL_UP);
    int last_vol_down = gpio_get_level(GPIO_BTN_VOL_DOWN);

    while (1) {
        const int doorbell = gpio_get_level(GPIO_BTN_DOORBELL);
        const int vol_up   = gpio_get_level(GPIO_BTN_VOL_UP);
        const int vol_down = gpio_get_level(GPIO_BTN_VOL_DOWN);

        if (doorbell && !last_doorbell) {
            vTaskDelay(pdMS_TO_TICKS(BUTTON_DEBOUNCE_MS));
            if (gpio_get_level(GPIO_BTN_DOORBELL)) {
                ESP_LOGI(GATTS_TABLE_TAG, "Doorbell pressed (GPIO%d)", GPIO_BTN_DOORBELL);
                send_doorbell_notify();          /* 1. Bluetooth signal first */
                audio_player_play_doorbell();    /* 2. then local I2S audio */
            }
        }

        if (vol_up && !last_vol_up) {
            vTaskDelay(pdMS_TO_TICKS(BUTTON_DEBOUNCE_MS));
            if (gpio_get_level(GPIO_BTN_VOL_UP)) {
                ESP_LOGI(GATTS_TABLE_TAG, "Volume up pressed (GPIO%d)", GPIO_BTN_VOL_UP);
                audio_player_volume_up();
            }
        }

        if (vol_down && !last_vol_down) {
            vTaskDelay(pdMS_TO_TICKS(BUTTON_DEBOUNCE_MS));
            if (gpio_get_level(GPIO_BTN_VOL_DOWN)) {
                ESP_LOGI(GATTS_TABLE_TAG, "Volume down pressed (GPIO%d)", GPIO_BTN_VOL_DOWN);
                audio_player_volume_down();
            }
        }

        last_doorbell = doorbell;
        last_vol_up   = vol_up;
        last_vol_down = vol_down;
        vTaskDelay(pdMS_TO_TICKS(BUTTON_POLL_MS));
    }
}

void app_main(void)
{
    esp_err_t ret;

    /* Initialize NVS */
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(GATTS_TABLE_TAG, "Doorbell slave starting (idle, connect-on-demand mode)");

    /* Timer: stop advertising if no host connects after a doorbell press */
    esp_timer_create_args_t adv_timeout_args = {
        .callback = adv_timeout_cb,
        .name = "adv_timeout",
    };
    ESP_ERROR_CHECK(esp_timer_create(&adv_timeout_args, &adv_timeout_timer));

    /* Timer: retry asynchronous advertising start/stop requests. */
    esp_timer_create_args_t adv_retry_args = {
        .callback = advertising_retry_timer_cb,
        .name = "adv_retry",
    };
    ESP_ERROR_CHECK(esp_timer_create(&adv_retry_args, &adv_retry_timer));

    /* Timer: close the short-lived link after notification delivery.  This
     * keeps the GATT callback non-blocking and lets the host receive the
     * notification before it reconnects for the next event. */
    esp_timer_create_args_t notify_close_args = {
        .callback = notify_close_timer_cb,
        .name = "notify_close",
    };
    ESP_ERROR_CHECK(esp_timer_create(&notify_close_args, &notify_close_timer));

    /* Buttons: GPIO16 doorbell, GPIO17 vol+, GPIO15 vol- (active high) */
    init_button_gpio();

    /* Audio player: embedded doorbell.wav -> I2S (MAX98357A) */
    ESP_ERROR_CHECK(audio_player_init());

    /* Button monitoring task */
    if (xTaskCreate(button_task, "buttons", BUTTON_TASK_STACK, NULL,
                    BUTTON_TASK_PRIO, NULL) != pdPASS) {
        ESP_LOGE(GATTS_TABLE_TAG, "Failed to create button task");
        return;
    }

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(GATTS_TABLE_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(GATTS_TABLE_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    esp_bluedroid_config_t cfg = BT_BLUEDROID_INIT_CONFIG_DEFAULT();
    ret = esp_bluedroid_init_with_cfg(&cfg);
    if (ret) {
        ESP_LOGE(GATTS_TABLE_TAG, "%s init bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(GATTS_TABLE_TAG, "%s enable bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    /* Use +9 dBm for advertisements and for packets on an established link. */
    ESP_ERROR_CHECK(esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, BLE_TX_POWER_LEVEL));
    ESP_ERROR_CHECK(esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, BLE_TX_POWER_LEVEL));

    ret = esp_ble_gatts_register_callback(gatts_event_handler);
    if (ret){
        ESP_LOGE(GATTS_TABLE_TAG, "gatts register error, error code = %x", ret);
        return;
    }

    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret){
        ESP_LOGE(GATTS_TABLE_TAG, "gap register error, error code = %x", ret);
        return;
    }

    ret = esp_ble_gatts_app_register(ESP_APP_ID);
    if (ret){
        ESP_LOGE(GATTS_TABLE_TAG, "gatts app register error, error code = %x", ret);
        return;
    }

    esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(500);
    if (local_mtu_ret){
        ESP_LOGE(GATTS_TABLE_TAG, "set local  MTU failed, error code = %x", local_mtu_ret);
    }
}
