/*
 * SPDX-FileCopyrightText: 2021-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

/****************************************************************************
*
* BLE GATT Client - Smart Doorbell Host
*
* Continuously scans for the doorbell slave device.
* On connection: enables notifications on characteristic 0xFF01.
* On notification 0x01: queues SD-card WAV playback and a TFT knock animation.
* After disconnect: resumes scanning.
*
****************************************************************************/

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include "nvs.h"
#include "nvs_flash.h"

#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#include "doorbell_media.h"

/* Physical doorbell button: GPIO15, pressed = high level. */
#define BUTTON_TRIGGER_GPIO      GPIO_NUM_15
#define BUTTON_DEBOUNCE_MS       50

/* Disable brownout detector for testing */
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

#define GATTC_TAG "DOORBELL_MASTER"
#define REMOTE_SERVICE_UUID        0x00FF
#define REMOTE_NOTIFY_CHAR_UUID    0xFF01
#define PROFILE_NUM      1
#define PROFILE_A_APP_ID 0
#define INVALID_HANDLE   0
#define INVALID_CONN_ID  UINT16_MAX
#define BLE_TX_POWER_LEVEL ESP_PWR_LVL_P9

#define SCAN_DURATION_S 30
#define SCAN_RETRY_DELAY_US (500 * 1000)
#define CONNECTION_SETUP_TIMEOUT_US (15 * 1000 * 1000)
#define CONNECTION_ABORT_GRACE_US (2 * 1000 * 1000)

typedef enum {
    SCAN_IDLE = 0,
    SCAN_STARTING,
    SCAN_ACTIVE,
    SCAN_STOPPING,
} scan_state_t;

static char remote_device_name[ESP_BLE_ADV_NAME_LEN_MAX] = "ESP_GATTS_DEMO";
static bool connect    = false;
static bool link_up    = false;
static bool get_server = false;
static scan_state_t scan_state = SCAN_IDLE;
static bool scan_wanted = false;
static bool scan_stop_requested = false;
static bool open_pending = false;
static bool open_in_flight = false;
static bool abort_waiting_for_disconnect = false;
static bool close_waiting_for_disconnect = false;
static esp_ble_gatt_creat_conn_params_t pending_conn_params;
static esp_timer_handle_t scan_retry_timer = NULL;
static esp_timer_handle_t connection_timeout_timer = NULL;

/* Declare static functions */
static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
static void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);
static void gattc_profile_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);
static void request_scan(void);
static void begin_pending_open(void);
static void schedule_scan_retry(void);
static void reset_discovery_state(void);
static void disarm_connection_timeout(void);

static esp_err_t configure_ble_tx_power(void)
{
    /* DEFAULT applies to new connections; SCAN covers active-scan requests. */
    esp_err_t ret = esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, BLE_TX_POWER_LEVEL);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_SCAN, BLE_TX_POWER_LEVEL);
    if (ret != ESP_OK) {
        return ret;
    }

    ESP_LOGI(GATTC_TAG, "BLE TX power set to +9 dBm (default and scan)");
    return ESP_OK;
}

static esp_bt_uuid_t remote_filter_service_uuid = {
    .len = ESP_UUID_LEN_16,
    .uuid = {.uuid16 = REMOTE_SERVICE_UUID,},
};

static esp_bt_uuid_t remote_filter_char_uuid = {
    .len = ESP_UUID_LEN_16,
    .uuid = {.uuid16 = REMOTE_NOTIFY_CHAR_UUID,},
};

static esp_bt_uuid_t notify_descr_uuid = {
    .len = ESP_UUID_LEN_16,
    .uuid = {.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG,},
};

static esp_ble_scan_params_t ble_scan_params = {
    .scan_type              = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type          = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy     = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval          = ESP_BLE_GAP_SCAN_ITVL_MS(50),
    .scan_window            = ESP_BLE_GAP_SCAN_WIN_MS(30),
    .scan_duplicate         = BLE_SCAN_DUPLICATE_DISABLE
};

struct gattc_profile_inst {
    esp_gattc_cb_t gattc_cb;
    uint16_t gattc_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_start_handle;
    uint16_t service_end_handle;
    uint16_t char_handle;
    esp_bd_addr_t remote_bda;
};

static struct gattc_profile_inst gl_profile_tab[PROFILE_NUM] = {
    [PROFILE_A_APP_ID] = {
        .gattc_cb = gattc_profile_event_handler,
        .gattc_if = ESP_GATT_IF_NONE,
        .conn_id = INVALID_CONN_ID,
        .service_start_handle = INVALID_HANDLE,
        .service_end_handle = INVALID_HANDLE,
        .char_handle = INVALID_HANDLE,
    },
};

static void scan_retry_timer_cb(void *arg)
{
    (void)arg;
    if (connect && scan_stop_requested && scan_state == SCAN_ACTIVE) {
        scan_state = SCAN_STOPPING;
        esp_err_t ret = esp_ble_gap_stop_scanning();
        if (ret != ESP_OK) {
            scan_state = SCAN_ACTIVE;
            ESP_LOGW(GATTC_TAG, "Retry scanning stop failed: %s",
                     esp_err_to_name(ret));
            schedule_scan_retry();
        }
        return;
    }

    if (connect && open_pending && scan_state == SCAN_IDLE) {
        scan_stop_requested = false;
        begin_pending_open();
        return;
    }

    if (!connect && scan_wanted && scan_state == SCAN_IDLE) {
        request_scan();
    }
}

static void connection_timeout_timer_cb(void *arg)
{
    (void)arg;

    if (!connect) {
        return;
    }

    if (close_waiting_for_disconnect) {
        close_waiting_for_disconnect = false;
        ESP_LOGW(GATTC_TAG,
                 "GATT close was not followed by disconnect; forcing physical disconnect");
        esp_err_t ret = esp_ble_gap_disconnect(
            gl_profile_tab[PROFILE_A_APP_ID].remote_bda);
        if (ret == ESP_OK) {
            abort_waiting_for_disconnect = true;
            if (connection_timeout_timer != NULL) {
                (void)esp_timer_start_once(connection_timeout_timer,
                                           CONNECTION_ABORT_GRACE_US);
            }
            return;
        }
        ESP_LOGW(GATTC_TAG, "Forced disconnect request failed: %s",
                 esp_err_to_name(ret));
        reset_discovery_state();
        scan_stop_requested = false;
        scan_wanted = true;
        request_scan();
        return;
    }

    if (abort_waiting_for_disconnect) {
        ESP_LOGW(GATTC_TAG, "Connection cleanup timed out; forcing scan recovery");
        abort_waiting_for_disconnect = false;
        reset_discovery_state();
        scan_stop_requested = false;
        scan_wanted = true;
        request_scan();
        return;
    }

    ESP_LOGW(GATTC_TAG, "BLE setup timed out; cancelling connection attempt");

    if (link_up) {
        esp_err_t ret = esp_ble_gap_disconnect(
            gl_profile_tab[PROFILE_A_APP_ID].remote_bda);
        if (ret == ESP_OK) {
            abort_waiting_for_disconnect = true;
            if (connection_timeout_timer != NULL) {
                (void)esp_timer_start_once(connection_timeout_timer,
                                           CONNECTION_ABORT_GRACE_US);
            }
            return;
        }
        ESP_LOGW(GATTC_TAG, "Timed-out link disconnect failed: %s",
                 esp_err_to_name(ret));
    } else if (open_in_flight) {
        esp_ble_gattc_cancel_open_params_t cancel_params = {
            .gattc_if = gl_profile_tab[PROFILE_A_APP_ID].gattc_if,
        };
        memcpy(cancel_params.remote_bda,
               gl_profile_tab[PROFILE_A_APP_ID].remote_bda,
               sizeof(cancel_params.remote_bda));
        esp_err_t ret = esp_ble_gattc_cancel_open(&cancel_params);
        if (ret == ESP_OK) {
            abort_waiting_for_disconnect = true;
            if (connection_timeout_timer != NULL) {
                (void)esp_timer_start_once(connection_timeout_timer,
                                           CONNECTION_ABORT_GRACE_US);
            }
            return;
        }
        ESP_LOGW(GATTC_TAG, "Timed-out open cancellation failed: %s",
                 esp_err_to_name(ret));
    }

    reset_discovery_state();
    scan_stop_requested = false;
    scan_wanted = true;
    request_scan();
}

static void schedule_scan_retry(void)
{
    if (scan_retry_timer == NULL ||
            (!scan_wanted && !(connect && scan_stop_requested))) {
        return;
    }

    (void)esp_timer_stop(scan_retry_timer);
    esp_err_t ret = esp_timer_start_once(scan_retry_timer, SCAN_RETRY_DELAY_US);
    if (ret != ESP_OK) {
        ESP_LOGW(GATTC_TAG, "Unable to schedule scan retry: %s",
                 esp_err_to_name(ret));
    }
}

static void arm_connection_timeout(void)
{
    if (connection_timeout_timer == NULL) {
        return;
    }

    abort_waiting_for_disconnect = false;
    (void)esp_timer_stop(connection_timeout_timer);
    esp_err_t ret = esp_timer_start_once(connection_timeout_timer,
                                         CONNECTION_SETUP_TIMEOUT_US);
    if (ret != ESP_OK) {
        ESP_LOGW(GATTC_TAG, "Unable to arm connection timeout: %s",
                 esp_err_to_name(ret));
    }
}

static void disarm_connection_timeout(void)
{
    if (connection_timeout_timer != NULL) {
        (void)esp_timer_stop(connection_timeout_timer);
    }
    abort_waiting_for_disconnect = false;
}

static void reset_discovery_state(void)
{
    connect = false;
    link_up = false;
    get_server = false;
    open_pending = false;
    open_in_flight = false;
    close_waiting_for_disconnect = false;
    disarm_connection_timeout();
    gl_profile_tab[PROFILE_A_APP_ID].conn_id = INVALID_CONN_ID;
    gl_profile_tab[PROFILE_A_APP_ID].service_start_handle = INVALID_HANDLE;
    gl_profile_tab[PROFILE_A_APP_ID].service_end_handle = INVALID_HANDLE;
    gl_profile_tab[PROFILE_A_APP_ID].char_handle = INVALID_HANDLE;

}

static void request_scan(void)
{
    scan_wanted = true;
    if (connect) {
        return;
    }

    if (scan_state != SCAN_IDLE) {
        return;
    }

    scan_state = SCAN_STARTING;
    esp_err_t ret = esp_ble_gap_start_scanning(SCAN_DURATION_S);
    if (ret == ESP_OK) {
        ESP_LOGI(GATTC_TAG, "Scanning start requested");
    } else {
        scan_state = SCAN_IDLE;
        ESP_LOGW(GATTC_TAG, "Scanning start failed: %s", esp_err_to_name(ret));
        schedule_scan_retry();
    }
}

static void begin_pending_open(void)
{
    if (!connect || !open_pending || open_in_flight ||
            scan_state != SCAN_IDLE || scan_stop_requested) {
        return;
    }

    open_pending = false;
    open_in_flight = true;
    esp_err_t ret = esp_ble_gattc_enh_open(
        gl_profile_tab[PROFILE_A_APP_ID].gattc_if, &pending_conn_params);
    if (ret != ESP_OK) {
        open_in_flight = false;
        ESP_LOGE(GATTC_TAG, "Connection request failed: %s",
                 esp_err_to_name(ret));
        reset_discovery_state();
        scan_stop_requested = false;
        request_scan();
    } else {
        ESP_LOGI(GATTC_TAG, "Connection request submitted");
    }
}

static void stop_scan_for_connection(void)
{
    scan_wanted = false;
    scan_stop_requested = true;

    if (scan_state == SCAN_IDLE) {
        scan_stop_requested = false;
        begin_pending_open();
        return;
    }

    if (scan_state == SCAN_STARTING || scan_state == SCAN_STOPPING) {
        return;
    }

    scan_state = SCAN_STOPPING;
    esp_err_t ret = esp_ble_gap_stop_scanning();
    if (ret != ESP_OK) {
        ESP_LOGW(GATTC_TAG, "Scanning stop request failed: %s",
                 esp_err_to_name(ret));
        /* The scan may still be active when the asynchronous stop request
         * fails. Retry it from the timer instead of opening concurrently. */
        scan_state = SCAN_ACTIVE;
        schedule_scan_retry();
    }
}

/* Abort discovery without leaving the central stuck in connect=true.  Once
 * an ACL link exists, let the normal DISCONNECT event restart scanning. */
static void abort_connection(const char *reason)
{
    ESP_LOGW(GATTC_TAG, "BLE setup failed (%s), recovering", reason);

    if (link_up) {
        esp_err_t ret = esp_ble_gap_disconnect(
            gl_profile_tab[PROFILE_A_APP_ID].remote_bda);
        if (ret == ESP_OK) {
            abort_waiting_for_disconnect = true;
            if (connection_timeout_timer != NULL) {
                (void)esp_timer_stop(connection_timeout_timer);
                (void)esp_timer_start_once(connection_timeout_timer,
                                           CONNECTION_ABORT_GRACE_US);
            }
            return;
        }
        ESP_LOGW(GATTC_TAG, "Physical disconnect request failed: %s",
                 esp_err_to_name(ret));
    } else if (open_in_flight) {
        esp_ble_gattc_cancel_open_params_t cancel_params = {
            .gattc_if = gl_profile_tab[PROFILE_A_APP_ID].gattc_if,
        };
        memcpy(cancel_params.remote_bda,
               gl_profile_tab[PROFILE_A_APP_ID].remote_bda,
               sizeof(cancel_params.remote_bda));
        esp_err_t ret = esp_ble_gattc_cancel_open(&cancel_params);
        if (ret == ESP_OK) {
            abort_waiting_for_disconnect = true;
            if (connection_timeout_timer != NULL) {
                (void)esp_timer_stop(connection_timeout_timer);
                (void)esp_timer_start_once(connection_timeout_timer,
                                           CONNECTION_ABORT_GRACE_US);
            }
            return;
        }
        ESP_LOGW(GATTC_TAG, "Pending connection cancellation failed: %s",
                 esp_err_to_name(ret));
    }

    reset_discovery_state();
    scan_stop_requested = false;
    request_scan();
}

static void gattc_profile_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
    esp_ble_gattc_cb_param_t *p_data = (esp_ble_gattc_cb_param_t *)param;

    switch (event) {
    case ESP_GATTC_REG_EVT:
        ESP_LOGI(GATTC_TAG, "GATT client register, status %d, app_id %d, gattc_if %d", param->reg.status, param->reg.app_id, gattc_if);
        if (param->reg.status != ESP_GATT_OK) {
            ESP_LOGE(GATTC_TAG, "GATT client registration failed");
            break;
        }
        esp_err_t scan_ret = esp_ble_gap_set_scan_params(&ble_scan_params);
        if (scan_ret){
            ESP_LOGE(GATTC_TAG, "set scan params error, error code = %x", scan_ret);
            scan_wanted = true;
            schedule_scan_retry();
        }
        break;
    case ESP_GATTC_CONNECT_EVT:{
        ESP_LOGI(GATTC_TAG, "Connected, conn_id %d, remote "ESP_BD_ADDR_STR"", p_data->connect.conn_id,
                 ESP_BD_ADDR_HEX(p_data->connect.remote_bda));
        if (!connect ||
                (!open_in_flight && !open_pending) ||
                memcmp(p_data->connect.remote_bda,
                       gl_profile_tab[PROFILE_A_APP_ID].remote_bda,
                       sizeof(esp_bd_addr_t)) != 0) {
            ESP_LOGW(GATTC_TAG, "Ignoring unexpected CONNECT event");
            break;
        }
        esp_err_t power_ret = esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_CONN_HDL0,
                                                   BLE_TX_POWER_LEVEL);
        if (power_ret != ESP_OK) {
            ESP_LOGW(GATTC_TAG, "Failed to set connection TX power to +9 dBm: %s",
                     esp_err_to_name(power_ret));
        } else {
            ESP_LOGI(GATTC_TAG, "Connection TX power set to +9 dBm");
        }
        close_waiting_for_disconnect = false;
        abort_waiting_for_disconnect = false;
        gl_profile_tab[PROFILE_A_APP_ID].conn_id = p_data->connect.conn_id;
        memcpy(gl_profile_tab[PROFILE_A_APP_ID].remote_bda, p_data->connect.remote_bda, sizeof(esp_bd_addr_t));
        link_up = true;
        open_in_flight = false;
        open_pending = false;
        get_server = false;
        gl_profile_tab[PROFILE_A_APP_ID].service_start_handle = INVALID_HANDLE;
        gl_profile_tab[PROFILE_A_APP_ID].service_end_handle = INVALID_HANDLE;
        gl_profile_tab[PROFILE_A_APP_ID].char_handle = INVALID_HANDLE;
        arm_connection_timeout();
        esp_err_t mtu_ret = esp_ble_gattc_send_mtu_req (gattc_if, p_data->connect.conn_id);
        if (mtu_ret){
            ESP_LOGW(GATTC_TAG, "Config MTU request failed: %s", esp_err_to_name(mtu_ret));
        }
        break;
    }
    case ESP_GATTC_OPEN_EVT:
        if (!connect ||
                memcmp(p_data->open.remote_bda,
                       gl_profile_tab[PROFILE_A_APP_ID].remote_bda,
                       sizeof(esp_bd_addr_t)) != 0 ||
                (gl_profile_tab[PROFILE_A_APP_ID].conn_id != INVALID_CONN_ID &&
                 p_data->open.conn_id != gl_profile_tab[PROFILE_A_APP_ID].conn_id) ||
                close_waiting_for_disconnect ||
                (!open_in_flight && !link_up)) {
            ESP_LOGW(GATTC_TAG, "Ignoring stale or unexpected OPEN event");
            break;
        }
        if (param->open.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "Open failed, status %d", p_data->open.status);
            abort_connection("open");
            break;
        }
        if (gl_profile_tab[PROFILE_A_APP_ID].conn_id == INVALID_CONN_ID) {
            gl_profile_tab[PROFILE_A_APP_ID].conn_id = p_data->open.conn_id;
        }
        open_in_flight = false;
        open_pending = false;
        link_up = true;
        abort_waiting_for_disconnect = false;
        arm_connection_timeout();
        ESP_LOGI(GATTC_TAG, "Open successfully, MTU %u", p_data->open.mtu);
        break;
    case ESP_GATTC_DIS_SRVC_CMPL_EVT:
        if (!connect || !link_up ||
                p_data->dis_srvc_cmpl.conn_id !=
                    gl_profile_tab[PROFILE_A_APP_ID].conn_id) {
            ESP_LOGW(GATTC_TAG, "Ignoring stale service discovery event");
            break;
        }
        if (param->dis_srvc_cmpl.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "Service discover failed, status %d", param->dis_srvc_cmpl.status);
            abort_connection("service discovery");
            break;
        }
        ESP_LOGI(GATTC_TAG, "Service discover complete, conn_id %d", param->dis_srvc_cmpl.conn_id);
        get_server = false;
        esp_err_t search_ret = esp_ble_gattc_search_service(
            gattc_if, param->dis_srvc_cmpl.conn_id, &remote_filter_service_uuid);
        if (search_ret != ESP_OK) {
            ESP_LOGE(GATTC_TAG, "Service search request failed: %s",
                     esp_err_to_name(search_ret));
            abort_connection("service search request");
        }
        break;
    case ESP_GATTC_CFG_MTU_EVT:
        ESP_LOGI(GATTC_TAG, "MTU exchange, status %d, MTU %d", param->cfg_mtu.status, param->cfg_mtu.mtu);
        break;
    case ESP_GATTC_SEARCH_RES_EVT: {
        if (!connect || !link_up ||
                p_data->search_res.conn_id !=
                    gl_profile_tab[PROFILE_A_APP_ID].conn_id) {
            ESP_LOGW(GATTC_TAG, "Ignoring stale service search result");
            break;
        }
        ESP_LOGI(GATTC_TAG, "Service search result, conn_id = %x, is primary service %d", p_data->search_res.conn_id, p_data->search_res.is_primary);
        if (p_data->search_res.srvc_id.uuid.len == ESP_UUID_LEN_16 && p_data->search_res.srvc_id.uuid.uuid.uuid16 == REMOTE_SERVICE_UUID) {
            ESP_LOGI(GATTC_TAG, "Service found");
            get_server = true;
            gl_profile_tab[PROFILE_A_APP_ID].service_start_handle = p_data->search_res.start_handle;
            gl_profile_tab[PROFILE_A_APP_ID].service_end_handle = p_data->search_res.end_handle;
        }
        break;
    }
    case ESP_GATTC_SEARCH_CMPL_EVT: {
        if (!connect || !link_up ||
                p_data->search_cmpl.conn_id !=
                    gl_profile_tab[PROFILE_A_APP_ID].conn_id) {
            ESP_LOGW(GATTC_TAG, "Ignoring stale service search completion");
            break;
        }
        if (p_data->search_cmpl.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "Service search failed, status %x", p_data->search_cmpl.status);
            abort_connection("service search");
            break;
        }
        ESP_LOGI(GATTC_TAG, "Service search complete");
        if (!get_server) {
            ESP_LOGE(GATTC_TAG, "Doorbell service 0x%04x not found",
                     REMOTE_SERVICE_UUID);
            abort_connection("doorbell service missing");
            break;
        }

        uint16_t count = 0;
        esp_gatt_status_t status = esp_ble_gattc_get_attr_count(
            gattc_if, p_data->search_cmpl.conn_id,
            ESP_GATT_DB_CHARACTERISTIC,
            gl_profile_tab[PROFILE_A_APP_ID].service_start_handle,
            gl_profile_tab[PROFILE_A_APP_ID].service_end_handle,
            INVALID_HANDLE, &count);
        if (status != ESP_GATT_OK) {
            ESP_LOGE(GATTC_TAG, "get characteristic count failed: %d", status);
            abort_connection("characteristic count");
            break;
        }
        if (count == 0) {
            ESP_LOGE(GATTC_TAG, "No characteristics in doorbell service");
            abort_connection("characteristic missing");
            break;
        }

        esp_gattc_char_elem_t *char_elem_result =
            malloc(sizeof(esp_gattc_char_elem_t) * count);
        if (char_elem_result == NULL) {
            ESP_LOGE(GATTC_TAG, "No memory for characteristic results");
            abort_connection("characteristic allocation");
            break;
        }

        status = esp_ble_gattc_get_char_by_uuid(
            gattc_if, p_data->search_cmpl.conn_id,
            gl_profile_tab[PROFILE_A_APP_ID].service_start_handle,
            gl_profile_tab[PROFILE_A_APP_ID].service_end_handle,
            remote_filter_char_uuid, char_elem_result, &count);
        if (status != ESP_GATT_OK) {
            ESP_LOGE(GATTC_TAG, "get notification characteristic failed: %d", status);
            free(char_elem_result);
            char_elem_result = NULL;
            abort_connection("notification characteristic lookup");
            break;
        }
        if (count == 0 ||
                !(char_elem_result[0].properties & ESP_GATT_CHAR_PROP_BIT_NOTIFY)) {
            ESP_LOGE(GATTC_TAG, "Characteristic 0x%04x has no notify property",
                     REMOTE_NOTIFY_CHAR_UUID);
            free(char_elem_result);
            char_elem_result = NULL;
            abort_connection("notification property missing");
            break;
        }

        gl_profile_tab[PROFILE_A_APP_ID].char_handle = char_elem_result[0].char_handle;
        esp_err_t reg_ret = esp_ble_gattc_register_for_notify(
            gattc_if, gl_profile_tab[PROFILE_A_APP_ID].remote_bda,
            gl_profile_tab[PROFILE_A_APP_ID].char_handle);
        free(char_elem_result);
        char_elem_result = NULL;
        if (reg_ret != ESP_OK) {
            ESP_LOGE(GATTC_TAG, "Register for notify failed: %s",
                     esp_err_to_name(reg_ret));
            abort_connection("notification registration");
        }
        break;
    }
    case ESP_GATTC_REG_FOR_NOTIFY_EVT: {
        if (!connect || !link_up ||
                gl_profile_tab[PROFILE_A_APP_ID].conn_id == INVALID_CONN_ID ||
                p_data->reg_for_notify.handle !=
                    gl_profile_tab[PROFILE_A_APP_ID].char_handle) {
            ESP_LOGW(GATTC_TAG, "Ignoring stale notify registration event");
            break;
        }
        if (p_data->reg_for_notify.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "Notification register failed, status %d", p_data->reg_for_notify.status);
            abort_connection("notification registration callback");
            break;
        }
        ESP_LOGI(GATTC_TAG, "Notification register successfully");

        uint16_t count = 0;
        esp_gatt_status_t count_status = esp_ble_gattc_get_attr_count(
            gattc_if, gl_profile_tab[PROFILE_A_APP_ID].conn_id,
            ESP_GATT_DB_DESCRIPTOR,
            gl_profile_tab[PROFILE_A_APP_ID].service_start_handle,
            gl_profile_tab[PROFILE_A_APP_ID].service_end_handle,
            gl_profile_tab[PROFILE_A_APP_ID].char_handle, &count);
        if (count_status != ESP_GATT_OK) {
            ESP_LOGE(GATTC_TAG, "Get descriptor count failed: %d", count_status);
            abort_connection("descriptor count");
            break;
        }
        if (count == 0) {
            ESP_LOGE(GATTC_TAG, "CCCD descriptor not found");
            abort_connection("CCCD missing");
            break;
        }

        esp_gattc_descr_elem_t *descr_elem_result =
            malloc(sizeof(esp_gattc_descr_elem_t) * count);
        if (descr_elem_result == NULL) {
            ESP_LOGE(GATTC_TAG, "No memory for descriptor results");
            abort_connection("descriptor allocation");
            break;
        }

        esp_gatt_status_t descr_status = esp_ble_gattc_get_descr_by_char_handle(
            gattc_if, gl_profile_tab[PROFILE_A_APP_ID].conn_id,
            p_data->reg_for_notify.handle, notify_descr_uuid,
            descr_elem_result, &count);
        if (descr_status != ESP_GATT_OK) {
            ESP_LOGE(GATTC_TAG, "Get CCCD failed: %d", descr_status);
            free(descr_elem_result);
            descr_elem_result = NULL;
            abort_connection("CCCD lookup");
            break;
        }
        if (count == 0 || descr_elem_result[0].uuid.len != ESP_UUID_LEN_16 ||
                descr_elem_result[0].uuid.uuid.uuid16 != ESP_GATT_UUID_CHAR_CLIENT_CONFIG) {
            ESP_LOGE(GATTC_TAG, "Returned descriptor is not a CCCD");
            free(descr_elem_result);
            descr_elem_result = NULL;
            abort_connection("invalid CCCD");
            break;
        }

        /* Explicit little-endian bytes are the on-air value 0x0001. */
        uint8_t notify_en[2] = {0x01, 0x00};
        esp_err_t write_ret = esp_ble_gattc_write_char_descr(
            gattc_if, gl_profile_tab[PROFILE_A_APP_ID].conn_id,
            descr_elem_result[0].handle, sizeof(notify_en), notify_en,
            ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_NONE);
        free(descr_elem_result);
        descr_elem_result = NULL;
        if (write_ret != ESP_OK) {
            ESP_LOGE(GATTC_TAG, "Write CCCD failed: %s", esp_err_to_name(write_ret));
            abort_connection("CCCD write request");
        }
        break;
    }
    case ESP_GATTC_NOTIFY_EVT:
        /* Doorbell notification received - hand the event to the media task. */
        if (!link_up || p_data->notify.conn_id !=
                gl_profile_tab[PROFILE_A_APP_ID].conn_id ||
                p_data->notify.handle != gl_profile_tab[PROFILE_A_APP_ID].char_handle ||
                p_data->notify.value == NULL) {
            ESP_LOGW(GATTC_TAG, "Ignoring stale or malformed notification");
            break;
        }
        if (p_data->notify.is_notify){
            ESP_LOGI(GATTC_TAG, "Notification received (doorbell trigger)");
        }else{
            ESP_LOGI(GATTC_TAG, "Indication received (doorbell trigger)");
        }
        ESP_LOG_BUFFER_HEX(GATTC_TAG, p_data->notify.value, p_data->notify.value_len);

        /* The doorbell protocol uses 0x01 as its trigger event. */
        if (p_data->notify.value_len >= 1 && p_data->notify.value[0] == 0x01) {
            disarm_connection_timeout();
            esp_err_t media_result = doorbell_media_trigger(p_data->notify.value[0]);
            if (media_result != ESP_OK) {
                ESP_LOGW(GATTC_TAG, "Doorbell media trigger not accepted: %s",
                         esp_err_to_name(media_result));
            } else {
                ESP_LOGI(GATTC_TAG, "Doorbell pressed; queued SD audio and knock animation");
            }
        }
        break;
    case ESP_GATTC_WRITE_DESCR_EVT:
        if (!connect || !link_up ||
                p_data->write.conn_id !=
                    gl_profile_tab[PROFILE_A_APP_ID].conn_id) {
            ESP_LOGW(GATTC_TAG, "Ignoring stale descriptor write event");
            break;
        }
        if (p_data->write.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "Descriptor write failed, status %x", p_data->write.status);
            abort_connection("CCCD write callback");
            break;
        }
        ESP_LOGI(GATTC_TAG, "Descriptor write successfully (notify enabled)");
        /* Keep the setup timeout armed until the deferred doorbell event is
         * actually received.  This also recovers if the server misses it. */
        /* Don't write to characteristic - we're just listening for notifications */
        break;
    case ESP_GATTC_SRVC_CHG_EVT: {
        esp_bd_addr_t bda;
        memcpy(bda, p_data->srvc_chg.remote_bda, sizeof(esp_bd_addr_t));
        ESP_LOGI(GATTC_TAG, "Service change from "ESP_BD_ADDR_STR"", ESP_BD_ADDR_HEX(bda));
        break;
    }
    case ESP_GATTC_WRITE_CHAR_EVT:
        if (p_data->write.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "Characteristic write failed, status %x)", p_data->write.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "Characteristic write successfully");
        break;
    case ESP_GATTC_DISCONNECT_EVT:
        if ((!connect && !link_up && !close_waiting_for_disconnect) ||
                memcmp(p_data->disconnect.remote_bda,
                       gl_profile_tab[PROFILE_A_APP_ID].remote_bda,
                       sizeof(esp_bd_addr_t)) != 0 ||
                (gl_profile_tab[PROFILE_A_APP_ID].conn_id != INVALID_CONN_ID &&
                 p_data->disconnect.conn_id !=
                     gl_profile_tab[PROFILE_A_APP_ID].conn_id)) {
            ESP_LOGW(GATTC_TAG, "Ignoring stale DISCONNECT event");
            break;
        }
        ESP_LOGI(GATTC_TAG, "Disconnected, remote "ESP_BD_ADDR_STR", reason 0x%02x",
                 ESP_BD_ADDR_HEX(p_data->disconnect.remote_bda), p_data->disconnect.reason);
        reset_discovery_state();
        scan_stop_requested = false;
        /* Resume scanning to find the doorbell again */
        ESP_LOGI(GATTC_TAG, "Resuming scan...");
        request_scan();
        break;
    case ESP_GATTC_CLOSE_EVT:
        if (!connect ||
                memcmp(p_data->close.remote_bda,
                       gl_profile_tab[PROFILE_A_APP_ID].remote_bda,
                       sizeof(esp_bd_addr_t)) != 0 ||
                (gl_profile_tab[PROFILE_A_APP_ID].conn_id != INVALID_CONN_ID &&
                 p_data->close.conn_id != gl_profile_tab[PROFILE_A_APP_ID].conn_id)) {
            ESP_LOGW(GATTC_TAG, "Ignoring stale GATT close event");
            break;
        }
        ESP_LOGI(GATTC_TAG, "GATT virtual connection closed, conn_id %u, status %d, reason %d",
                 p_data->close.conn_id, p_data->close.status, p_data->close.reason);
        /* With one GATT client a virtual close normally leads to a physical
         * disconnect.  Give that event a short grace period, then force the
         * ACL down so the central cannot remain stuck in connect=true. */
        link_up = false;
        open_pending = false;
        open_in_flight = false;
        get_server = false;
        close_waiting_for_disconnect = true;
        if (connection_timeout_timer != NULL) {
            (void)esp_timer_stop(connection_timeout_timer);
            esp_err_t timer_ret = esp_timer_start_once(
                connection_timeout_timer, CONNECTION_ABORT_GRACE_US);
            if (timer_ret != ESP_OK) {
                ESP_LOGW(GATTC_TAG, "Unable to wait for disconnect: %s",
                         esp_err_to_name(timer_ret));
                close_waiting_for_disconnect = false;
                reset_discovery_state();
                scan_stop_requested = false;
                scan_wanted = true;
                request_scan();
            }
        }
        break;
    case ESP_GATTC_CANCEL_OPEN_EVT:
        if (!connect || link_up || (!open_in_flight && !abort_waiting_for_disconnect) ||
                memcmp(p_data->cancel_open.remote_bda,
                       gl_profile_tab[PROFILE_A_APP_ID].remote_bda,
                       sizeof(esp_bd_addr_t)) != 0) {
            ESP_LOGW(GATTC_TAG, "Ignoring stale CANCEL_OPEN event");
            break;
        }
        ESP_LOGW(GATTC_TAG, "Pending connection cancelled, status %d",
                 p_data->cancel_open.status);
        reset_discovery_state();
        scan_stop_requested = false;
        request_scan();
        break;
    default:
        break;
    }
}

static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    uint8_t *adv_name = NULL;
    uint8_t adv_name_len = 0;
    switch (event) {
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT: {
        if (param->scan_param_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(GATTC_TAG, "Scan parameter setup failed, status %x",
                     param->scan_param_cmpl.status);
            scan_state = SCAN_IDLE;
            scan_wanted = true;
            schedule_scan_retry();
            break;
        }
        request_scan();
        break;
    }
    case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
        if (scan_state != SCAN_STARTING) {
            ESP_LOGW(GATTC_TAG, "Ignoring stale scan-start completion");
            break;
        }
        if (param->scan_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            scan_state = SCAN_IDLE;
            ESP_LOGE(GATTC_TAG, "Scanning start failed, status %x", param->scan_start_cmpl.status);
            if (connect && scan_stop_requested) {
                scan_stop_requested = false;
                begin_pending_open();
            } else if (!connect && scan_wanted) {
                schedule_scan_retry();
            }
            break;
        }
        scan_state = SCAN_ACTIVE;
        scan_wanted = false;
        ESP_LOGI(GATTC_TAG, "Scanning start successfully");
        if (connect && scan_stop_requested) {
            stop_scan_for_connection();
        }
        break;
    case ESP_GAP_BLE_SCAN_RESULT_EVT: {
        esp_ble_gap_cb_param_t *scan_result = (esp_ble_gap_cb_param_t *)param;
        switch (scan_result->scan_rst.search_evt) {
        case ESP_GAP_SEARCH_INQ_RES_EVT:
            if (scan_state != SCAN_ACTIVE || connect) {
                break;
            }
            adv_name = esp_ble_resolve_adv_data_by_type(scan_result->scan_rst.ble_adv,
                                                        scan_result->scan_rst.adv_data_len + scan_result->scan_rst.scan_rsp_len,
                                                        ESP_BLE_AD_TYPE_NAME_CMPL,
                                                        &adv_name_len);

            if (adv_name != NULL) {
                if (strlen(remote_device_name) == adv_name_len && strncmp((char *)adv_name, remote_device_name, adv_name_len) == 0) {
                    ESP_LOGI(GATTC_TAG, "Doorbell device found: %s", remote_device_name);
                    if (connect == false) {
                        connect = true;
                        scan_wanted = false;
                        memset(&pending_conn_params, 0, sizeof(pending_conn_params));
                        memcpy(pending_conn_params.remote_bda,
                               scan_result->scan_rst.bda,
                               ESP_BD_ADDR_LEN);
                        memcpy(gl_profile_tab[PROFILE_A_APP_ID].remote_bda,
                               scan_result->scan_rst.bda,
                               sizeof(esp_bd_addr_t));
                        gl_profile_tab[PROFILE_A_APP_ID].conn_id = INVALID_CONN_ID;
                        pending_conn_params.remote_addr_type =
                            scan_result->scan_rst.ble_addr_type;
                        pending_conn_params.own_addr_type = BLE_ADDR_TYPE_PUBLIC;
                        pending_conn_params.is_direct = true;
                        pending_conn_params.is_aux = false;
                        pending_conn_params.phy_mask = 0x0;
                        open_pending = true;
                        arm_connection_timeout();
                        ESP_LOGI(GATTC_TAG, "Connecting to doorbell...");
                        stop_scan_for_connection();
                    }
                }
            }
            break;
        case ESP_GAP_SEARCH_INQ_CMPL_EVT:
            /* Scan timeout - restart scanning */
            if (scan_state == SCAN_ACTIVE) {
                scan_state = SCAN_IDLE;
                ESP_LOGW(GATTC_TAG, "Scan timeout, restarting...");
                if (connect && open_pending) {
                    scan_stop_requested = false;
                    begin_pending_open();
                } else if (!connect) {
                    request_scan();
                }
            }
            break;
        default:
            break;
        }
        break;
    }

    case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
        if (scan_state != SCAN_STOPPING) {
            ESP_LOGW(GATTC_TAG, "Ignoring stale scan-stop completion");
            break;
        }
        scan_state = SCAN_IDLE;
        if (param->scan_stop_cmpl.status != ESP_BT_STATUS_SUCCESS){
            ESP_LOGE(GATTC_TAG, "Scanning stop failed, status %x", param->scan_stop_cmpl.status);
            scan_stop_requested = false;
            if (connect) {
                abort_connection("scanning stop");
            } else if (scan_wanted) {
                request_scan();
            }
        } else {
            ESP_LOGI(GATTC_TAG, "Scanning stop successfully");
            scan_stop_requested = false;
            if (connect && open_pending) {
                begin_pending_open();
            } else if (!connect && scan_wanted) {
                request_scan();
            }
        }
        break;

    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS){
            ESP_LOGE(GATTC_TAG, "Advertising stop failed, status %x", param->adv_stop_cmpl.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "Advertising stop successfully");
        break;
    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
         ESP_LOGI(GATTC_TAG, "Connection params update, status %d, conn_int %d, latency %d, timeout %d",
                  param->update_conn_params.status,
                  param->update_conn_params.conn_int,
                  param->update_conn_params.latency,
                  param->update_conn_params.timeout);
        break;
    case ESP_GAP_BLE_SET_PKT_LENGTH_COMPLETE_EVT:
        ESP_LOGI(GATTC_TAG, "Packet length update, status %d, rx %d, tx %d",
                  param->pkt_data_length_cmpl.status,
                  param->pkt_data_length_cmpl.params.rx_len,
                  param->pkt_data_length_cmpl.params.tx_len);
        break;
    default:
        break;
    }
}

static void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
    if (event == ESP_GATTC_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            gl_profile_tab[param->reg.app_id].gattc_if = gattc_if;
        } else {
            ESP_LOGI(GATTC_TAG, "reg app failed, app_id %04x, status %d",
                    param->reg.app_id,
                    param->reg.status);
            return;
        }
    }

    do {
        int idx;
        for (idx = 0; idx < PROFILE_NUM; idx++) {
            if (gattc_if == ESP_GATT_IF_NONE ||
                    gattc_if == gl_profile_tab[idx].gattc_if) {
                if (gl_profile_tab[idx].gattc_cb) {
                    gl_profile_tab[idx].gattc_cb(event, gattc_if, param);
                }
            }
        }
    } while (0);
}

/* GPIO button trigger: a rising edge on BUTTON_TRIGGER_GPIO (pressed = high)
 * plays the doorbell audio, with software debouncing. */
static void button_trigger_task(void *arg)
{
    (void)arg;
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << BUTTON_TRIGGER_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    if (gpio_config(&io) != ESP_OK) {
        ESP_LOGE(GATTC_TAG, "Button GPIO config failed");
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(GATTC_TAG, "Button trigger ready: GPIO%d pressed=high plays doorbell audio",
             BUTTON_TRIGGER_GPIO);

    uint32_t stable_ticks = 0;
    bool pressed = false;
    while (true) {
        bool level = gpio_get_level(BUTTON_TRIGGER_GPIO) != 0;
        if (level) {
            /* Rising edge - require the level to stay high for the debounce
             * window before treating it as a press. */
            if (!pressed && ++stable_ticks >= pdMS_TO_TICKS(BUTTON_DEBOUNCE_MS)) {
                pressed = true;
                esp_err_t ret = doorbell_media_trigger(0x01);
                if (ret == ESP_OK) {
                    ESP_LOGI(GATTC_TAG, "Button trigger: doorbell audio + knock animation queued");
                } else {
                    ESP_LOGW(GATTC_TAG, "Button trigger not accepted: %s", esp_err_to_name(ret));
                }
            }
        } else {
            stable_ticks = 0;
            pressed = false;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/* Serial console trigger: typing '1' (or 'p') in the monitor plays the
 * doorbell audio without needing the BLE doorbell slave device.
 *
 * Note: in ESP-IDF v6 the console UART has no driver installed, so the VFS
 * read is non-blocking (returns EWOULDBLOCK when idle). Polling without a
 * delay would spin the CPU and trip the task watchdog, so yield each round. */
static void serial_trigger_task(void *arg)
{
    (void)arg;
    setvbuf(stdin, NULL, _IONBF, 0);
    ESP_LOGI(GATTC_TAG, "Serial trigger ready: send '1' in the monitor to play doorbell audio");

    while (true) {
        int c = fgetc(stdin);
        if (c == '1' || c == 'p' || c == 'P') {
            esp_err_t ret = doorbell_media_trigger(0x01);
            if (ret == ESP_OK) {
                ESP_LOGI(GATTC_TAG, "Serial trigger: doorbell audio + knock animation queued");
            } else {
                ESP_LOGW(GATTC_TAG, "Serial trigger not accepted: %s", esp_err_to_name(ret));
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void app_main(void)
{
    /* Disable brownout detector (for testing only - fix power supply for production) */
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

    esp_err_t ret;

    /* Initialize NVS */
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    const esp_timer_create_args_t scan_retry_args = {
        .callback = scan_retry_timer_cb,
        .name = "gattc_scan_retry",
    };
    ESP_ERROR_CHECK(esp_timer_create(&scan_retry_args, &scan_retry_timer));

    const esp_timer_create_args_t connection_timeout_args = {
        .callback = connection_timeout_timer_cb,
        .name = "gattc_conn_timeout",
    };
    ESP_ERROR_CHECK(esp_timer_create(&connection_timeout_args,
                                     &connection_timeout_timer));

    /* Start the display/SD/audio subsystem before enabling BLE notifications. */
    ret = doorbell_media_init();
    if (ret != ESP_OK) {
        ESP_LOGE(GATTC_TAG, "Media subsystem init failed: %s", esp_err_to_name(ret));
        return;
    }

    /* Serial console trigger so the doorbell audio can be tested without the
     * BLE slave device: type '1' (or 'p') in the monitor. */
    if (xTaskCreatePinnedToCore(serial_trigger_task, "serial_trigger", 4096,
                                NULL, 3, NULL, 0) != pdPASS) {
        ESP_LOGE(GATTC_TAG, "Cannot create serial trigger task");
    }

    /* Physical doorbell button on GPIO15 (pressed = high). */
    if (xTaskCreatePinnedToCore(button_trigger_task, "button_trigger", 4096,
                                NULL, 3, NULL, 0) != pdPASS) {
        ESP_LOGE(GATTC_TAG, "Cannot create button trigger task");
    }

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(GATTC_TAG, "%s initialize controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(GATTC_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    esp_bluedroid_config_t cfg = BT_BLUEDROID_INIT_CONFIG_DEFAULT();
    ret = esp_bluedroid_init_with_cfg(&cfg);
    if (ret) {
        ESP_LOGE(GATTC_TAG, "%s init bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(GATTC_TAG, "%s enable bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = configure_ble_tx_power();
    if (ret != ESP_OK) {
        ESP_LOGE(GATTC_TAG, "Failed to set BLE TX power to +9 dBm: %s",
                 esp_err_to_name(ret));
        return;
    }

    ret = esp_ble_gap_register_callback(esp_gap_cb);
    if (ret){
        ESP_LOGE(GATTC_TAG, "%s gap register failed, error code = %x", __func__, ret);
        return;
    }

    ret = esp_ble_gattc_register_callback(esp_gattc_cb);
    if(ret){
        ESP_LOGE(GATTC_TAG, "%s gattc register failed, error code = %x", __func__, ret);
        return;
    }

    ret = esp_ble_gattc_app_register(PROFILE_A_APP_ID);
    if (ret){
        ESP_LOGE(GATTC_TAG, "%s gattc app register failed, error code = %x", __func__, ret);
    }

    esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(500);
    if (local_mtu_ret){
        ESP_LOGE(GATTC_TAG, "set local  MTU failed, error code = %x", local_mtu_ret);
    }

    ESP_LOGI(GATTC_TAG, "Doorbell master started. Waiting for doorbell device...");
}
