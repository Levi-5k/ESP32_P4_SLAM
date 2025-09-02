/*
 * AssistNow Offline Component
 * Downloads and manages u-blox AssistNow Offline satellite ephemeris data
 */

#pragma once

#include <esp_err.h>
#include <stdbool.h>
#include <stdint.h>
#include <driver/uart.h>
#include "wifi_assistnow.h"

#ifdef __cplusplus
extern "C" {
#endif

// AssistNow data types
typedef enum {
    ASSISTNOW_STATUS_DISABLED = 0,
    ASSISTNOW_STATUS_NO_DATA,
    ASSISTNOW_STATUS_DATA_VALID,
    ASSISTNOW_STATUS_DATA_EXPIRED,
    ASSISTNOW_STATUS_DOWNLOADING,
    ASSISTNOW_STATUS_ERROR
} assistnow_status_t;

// AssistNow configuration structure
typedef struct {
    bool enabled;
    char token[64];
    uint32_t update_interval_hours;
    uint32_t validity_hours;
    bool auto_download;
    char server_url[128];
    char data_file_path[256];
    // Enhanced URL parameters for compatibility with u-blox software
    char gnss_constellations[32];  // e.g., "gps,glo,gal,bds"
    uint32_t period_days;         // validity period in days
    uint32_t resolution_hours;    // time resolution
} assistnow_config_t;

// AssistNow data information
typedef struct {
    assistnow_status_t status;
    uint64_t download_timestamp;
    uint64_t expiry_timestamp;
    uint32_t data_size_bytes;
    uint32_t satellites_count;
    char version[32];
    bool loaded_in_gps;
} assistnow_info_t;

// Function declarations
esp_err_t assistnow_init(const assistnow_config_t* config);
esp_err_t assistnow_deinit(void);
esp_err_t assistnow_download_data(void);
esp_err_t assistnow_upload_to_gps(void);
esp_err_t assistnow_upload_to_ublox(uart_port_t uart_port);
esp_err_t assistnow_get_info(assistnow_info_t* info);
esp_err_t assistnow_force_download(void);
esp_err_t assistnow_clear_data(void);
bool assistnow_is_data_valid(void);
bool assistnow_is_download_needed(void);
esp_err_t assistnow_set_config(const assistnow_config_t* config);

/**
 * @brief Set WiFi configuration for AssistNow downloads
 * @param wifi_config WiFi configuration with SSID, password, and connection settings
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG if config is NULL
 * @note This function should be called after assistnow_init() to provide WiFi credentials
 *       for downloading AssistNow data from u-blox servers
 */
esp_err_t assistnow_set_wifi_config(const wifi_assistnow_config_t* wifi_config);

// Background task management
esp_err_t assistnow_start_auto_update(void);
esp_err_t assistnow_stop_auto_update(void);

// Data management
esp_err_t assistnow_save_to_storage(const uint8_t* data, size_t size);
esp_err_t assistnow_load_from_storage(uint8_t* data, size_t* size);
esp_err_t assistnow_delete_from_storage(void);


#ifdef __cplusplus
}
#endif
