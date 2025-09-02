/*
 * WiFi Positioning Component Implementation
 * Uses WiGLE database and WiFi scanning for position estimation
 * ESP32-P4 uses WiFi remote over SDIO interface
 */

#include "wifi_positioning.h"
#include <esp_log.h>
#include <esp_wifi.h>  // For WiFi types
#include <esp_wifi_remote_api.h>  // For remote WiFi functions
#include <esp_event.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>
#include <string.h>
#include <math.h>
#include "sd_storage.h"
// #include "wigle_db.h"  // Temporarily disabled for offline-only operation

#define WIGLE_DB_AVAILABLE 0  // Set to 1 when wigle_db component is working

static const char *TAG = "WIFI_POSITIONING";

// Stub implementations for when wigle_db is not available
#if WIGLE_DB_AVAILABLE == 0
typedef struct {
    char bssid[18];
    double latitude;
    double longitude;
    float altitude;
    uint32_t last_update;
    uint8_t signal_samples;
    int8_t avg_rssi;
    uint16_t accuracy;
} wigle_db_entry_t;

typedef struct {
    char database_path[256];
    uint32_t max_entries;
    bool enable_compression;
    uint32_t cache_size;
} wigle_db_config_t;

typedef struct {
    uint32_t total_entries;
    uint32_t valid_entries;
    uint32_t duplicate_entries;
    uint64_t last_update_timestamp;
    uint32_t database_version;
    char source_file[256];
} wigle_db_stats_t;

static esp_err_t wigle_db_init(const wigle_db_config_t* config) {
    ESP_LOGW(TAG, "WiGLE database not available - offline positioning only");
    return ESP_ERR_NOT_SUPPORTED;
}

static esp_err_t wigle_db_load_from_file(const char* file_path) {
    ESP_LOGW(TAG, "WiGLE database not available - cannot load from file");
    return ESP_ERR_NOT_SUPPORTED;
}

static esp_err_t wigle_db_get_stats(wigle_db_stats_t* stats) {
    if (stats) {
        memset(stats, 0, sizeof(wigle_db_stats_t));
    }
    return ESP_ERR_NOT_SUPPORTED;
}

static esp_err_t wigle_db_find_by_bssid(const char* bssid, wigle_db_entry_t* entry) {
    ESP_LOGD(TAG, "WiGLE database not available - cannot find BSSID: %s", bssid);
    return ESP_ERR_NOT_FOUND;
}

static esp_err_t wigle_db_deinit(void) {
    return ESP_OK;
}
#endif

// Component state
static bool initialized = false;
static wifi_positioning_config_t config;
static wifi_positioning_status_t status = WIFI_POS_STATUS_DISABLED;
static wifi_positioning_stats_t stats = {0};
static wifi_position_t last_position = {0};

// WiGLE database
static wigle_db_config_t wigle_config = {
    .database_path = "/sdcard/wigle/wigle.db",
    .max_entries = 10000,
    .enable_compression = false,
    .cache_size = 1024
};

// Task and synchronization
static TaskHandle_t positioning_task = NULL;
static EventGroupHandle_t event_group = NULL;
static bool auto_scan_enabled = false;

// Event group bits
#define SCAN_COMPLETE_BIT     BIT0
#define POSITION_READY_BIT    BIT1
#define STOP_REQUEST_BIT      BIT2

// Constants
#define WIFI_SCAN_TIMEOUT_MS  10000
#define MAX_VISIBLE_APS       50
#define EARTH_RADIUS_M        6371000.0f
#define WIFI_POS_TASK_STACK   4096
#define WIFI_POS_TASK_PRIORITY 5

// Forward declarations
static void positioning_task_func(void* pvParameters);
static esp_err_t perform_wifi_scan(wifi_ap_info_t* aps, uint8_t* count, uint8_t max_count);
static esp_err_t calculate_position_from_aps(const wifi_ap_info_t* aps, uint8_t ap_count, wifi_position_t* position);
static wigle_entry_t* find_wigle_entry(const char* bssid);
static double calculate_distance_weighted_position(const wigle_entry_t* entries[], uint8_t entry_count,
                                                  const int8_t rssis[], wifi_position_t* position);
static double haversine_distance(double lat1, double lon1, double lat2, double lon2);

/**
 * @brief Initialize WiFi positioning system
 */
esp_err_t wifi_positioning_init(const wifi_positioning_config_t* config_param)
{
    if (initialized) {
        ESP_LOGW(TAG, "WiFi positioning already initialized");
        return ESP_OK;
    }

    if (!config_param) {
        ESP_LOGE(TAG, "Invalid configuration");
        return ESP_ERR_INVALID_ARG;
    }

    // Copy configuration
    memcpy(&config, config_param, sizeof(wifi_positioning_config_t));

    // Set defaults if not provided
    if (config.scan_interval_ms == 0) config.scan_interval_ms = 30000; // 30 seconds
    if (config.min_ap_count == 0) config.min_ap_count = 3;
    if (config.max_ap_count == 0) config.max_ap_count = 10;
    if (config.min_rssi_threshold == 0) config.min_rssi_threshold = -85;
    if (config.position_timeout_ms == 0) config.position_timeout_ms = 5000;

    // Set default database path if not provided
    if (strlen(config.wigle_db_path) == 0) {
        strcpy(config.wigle_db_path, "/sdcard/wigle/wigle.db");
    }

    ESP_LOGI(TAG, "Initializing WiFi positioning system");
    ESP_LOGI(TAG, "Config: scan_interval=%u ms, min_ap=%u, max_ap=%u, min_rssi=%d",
             config.scan_interval_ms, config.min_ap_count, config.max_ap_count, config.min_rssi_threshold);

    // Create event group
    event_group = xEventGroupCreate();
    if (!event_group) {
        ESP_LOGE(TAG, "Failed to create event group");
        return ESP_ERR_NO_MEM;
    }

    // Initialize WiGLE database
    wigle_config.max_entries = config.wigle_db_size > 0 ? config.wigle_db_size : 10000;
    strcpy(wigle_config.database_path, config.wigle_db_path);

    // Create the wigle directory if it doesn't exist
    char wigle_dir[256];
    strcpy(wigle_dir, config.wigle_db_path);
    char* last_slash = strrchr(wigle_dir, '/');
    if (last_slash) {
        *last_slash = '\0';  // Remove the filename part, keep directory
        esp_err_t dir_ret = sd_storage_create_directory(wigle_dir);
        if (dir_ret != ESP_OK && dir_ret != ESP_ERR_INVALID_STATE) {
            ESP_LOGW(TAG, "Failed to create WiGLE directory %s: %s", wigle_dir, esp_err_to_name(dir_ret));
            // Continue anyway - the database init might still work
        } else {
            ESP_LOGI(TAG, "✅ Created WiGLE directory: %s", wigle_dir);
        }
    }

    esp_err_t wigle_ret = wigle_db_init(&wigle_config);
    if (wigle_ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to initialize WiGLE database: %s", esp_err_to_name(wigle_ret));
        ESP_LOGW(TAG, "WiFi positioning will work with online fallback only");
        // Don't return error - continue with online-only mode
    } else {
        // Load WiGLE database
        esp_err_t ret = wigle_db_load_from_file(config.wigle_db_path);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to load WiGLE database: %s", esp_err_to_name(ret));
            ESP_LOGW(TAG, "WiFi positioning will work with online fallback");
        } else {
            wigle_db_stats_t stats;
            wigle_db_get_stats(&stats);
            ESP_LOGI(TAG, "Loaded WiGLE database with %u entries", stats.valid_entries);
        }
    }

    status = WIFI_POS_STATUS_READY;
    initialized = true;

    ESP_LOGI(TAG, "✅ WiFi positioning initialized successfully");

    return ESP_OK;
}

/**
 * @brief Deinitialize WiFi positioning system
 */
esp_err_t wifi_positioning_deinit(void)
{
    if (!initialized) {
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Deinitializing WiFi positioning system");

    // Stop positioning task
    wifi_positioning_stop();

    // Free WiGLE database
    wigle_db_deinit();

    // Delete event group
    if (event_group) {
        vEventGroupDelete(event_group);
        event_group = NULL;
    }

    status = WIFI_POS_STATUS_DISABLED;
    initialized = false;

    ESP_LOGI(TAG, "✅ WiFi positioning deinitialized");

    return ESP_OK;
}

/**
 * @brief Start WiFi scanning and positioning
 */
esp_err_t wifi_positioning_start(void)
{
    if (!initialized) {
        ESP_LOGE(TAG, "WiFi positioning not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (positioning_task) {
        ESP_LOGW(TAG, "Positioning task already running");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Starting WiFi positioning task");

    // Create positioning task
    BaseType_t ret = xTaskCreate(
        positioning_task_func,
        "wifi_positioning",
        WIFI_POS_TASK_STACK,
        NULL,
        WIFI_POS_TASK_PRIORITY,
        &positioning_task
    );

    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create positioning task");
        return ESP_ERR_NO_MEM;
    }

    auto_scan_enabled = true;
    status = WIFI_POS_STATUS_SCANNING;

    ESP_LOGI(TAG, "✅ WiFi positioning started");

    return ESP_OK;
}

/**
 * @brief Stop WiFi scanning and positioning
 */
esp_err_t wifi_positioning_stop(void)
{
    if (!initialized) {
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Stopping WiFi positioning");

    auto_scan_enabled = false;

    // Signal task to stop
    if (event_group) {
        xEventGroupSetBits(event_group, STOP_REQUEST_BIT);
    }

    // Wait for task to finish
    if (positioning_task) {
        vTaskDelay(pdMS_TO_TICKS(100)); // Give task time to clean up
        vTaskDelete(positioning_task);
        positioning_task = NULL;
    }

    status = WIFI_POS_STATUS_READY;

    ESP_LOGI(TAG, "✅ WiFi positioning stopped");

    return ESP_OK;
}

/**
 * @brief WiFi positioning task function
 */
static void positioning_task_func(void* pvParameters)
{
    ESP_LOGI(TAG, "WiFi positioning task started");

    while (auto_scan_enabled) {
        // Wait for scan interval or stop request
        EventBits_t bits = xEventGroupWaitBits(
            event_group,
            STOP_REQUEST_BIT,
            pdTRUE,  // Clear bits on exit
            pdFALSE, // Don't wait for all bits
            pdMS_TO_TICKS(config.scan_interval_ms)
        );

        if (bits & STOP_REQUEST_BIT) {
            break; // Exit task
        }

        // Perform scan and position calculation
        wifi_position_t position;
        esp_err_t ret = wifi_positioning_scan_and_calculate(&position);

        if (ret == ESP_OK && position.valid) {
            last_position = position;
            stats.successful_positions++;
            xEventGroupSetBits(event_group, POSITION_READY_BIT);
        } else {
            stats.failed_positions++;
        }

        stats.total_scans++;
    }

    ESP_LOGI(TAG, "WiFi positioning task stopped");
    positioning_task = NULL;
    vTaskDelete(NULL);
}

/**
 * @brief Perform single WiFi scan and position calculation
 */
esp_err_t wifi_positioning_scan_and_calculate(wifi_position_t* position)
{
    if (!initialized || !position) {
        return ESP_ERR_INVALID_ARG;
    }

    status = WIFI_POS_STATUS_SCANNING;

    // Scan for WiFi networks
    wifi_ap_info_t aps[MAX_VISIBLE_APS];
    uint8_t ap_count = 0;

    esp_err_t ret = perform_wifi_scan(aps, &ap_count, MAX_VISIBLE_APS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "WiFi scan failed: %s", esp_err_to_name(ret));
        status = WIFI_POS_STATUS_ERROR;
        return ret;
    }

    stats.last_scan_ap_count = ap_count;

    if (ap_count < config.min_ap_count) {
        ESP_LOGW(TAG, "Not enough APs found: %u (minimum: %u)", ap_count, config.min_ap_count);
        status = WIFI_POS_STATUS_READY;
        return ESP_ERR_NOT_FOUND;
    }

    ESP_LOGI(TAG, "Found %u WiFi access points", ap_count);

    // Calculate position
    status = WIFI_POS_STATUS_CALCULATING;
    ret = calculate_position_from_aps(aps, ap_count, position);

    if (ret == ESP_OK) {
        position->timestamp = esp_timer_get_time() / 1000; // microseconds to milliseconds
        ESP_LOGI(TAG, "WiFi position calculated: %.6f, %.6f (accuracy: %.1fm)",
                 position->latitude, position->longitude, position->accuracy_h);
    }

    status = WIFI_POS_STATUS_READY;
    return ret;
}

/**
 * @brief Perform WiFi scan
 */
static esp_err_t perform_wifi_scan(wifi_ap_info_t* aps, uint8_t* count, uint8_t max_count)
{
    // Initialize WiFi remote for ESP32-P4 (SDIO interface)
    static bool wifi_initialized = false;
    if (!wifi_initialized) {
        // Initialize WiFi remote (ESP32-P4 uses SDIO interface)
        ESP_ERROR_CHECK(esp_wifi_remote_init(NULL));  // Use default config
        ESP_ERROR_CHECK(esp_wifi_remote_set_mode(WIFI_MODE_STA));
        ESP_ERROR_CHECK(esp_wifi_remote_start());
        wifi_initialized = true;
    }

    // Configure scan
    wifi_scan_config_t scan_config = {
        .ssid = NULL,
        .bssid = NULL,
        .channel = 0,
        .show_hidden = true,
        .scan_type = WIFI_SCAN_TYPE_ACTIVE,
        .scan_time.active.min = 100,
        .scan_time.active.max = 300
    };

    // Start scan
    ESP_ERROR_CHECK(esp_wifi_remote_scan_start(&scan_config, true));

    // Get scan results
    uint16_t ap_num = 0;
    ESP_ERROR_CHECK(esp_wifi_remote_scan_get_ap_num(&ap_num));

    if (ap_num == 0) {
        *count = 0;
        return ESP_OK;
    }

    // Allocate memory for scan results
    wifi_ap_record_t* ap_records = (wifi_ap_record_t*)malloc(ap_num * sizeof(wifi_ap_record_t));
    if (!ap_records) {
        ESP_LOGE(TAG, "Failed to allocate memory for AP records");
        return ESP_ERR_NO_MEM;
    }

    ESP_ERROR_CHECK(esp_wifi_remote_scan_get_ap_records(&ap_num, ap_records));

    // Process results
    uint8_t valid_count = 0;
    for (uint16_t i = 0; i < ap_num && valid_count < max_count; i++) {
        wifi_ap_record_t* record = &ap_records[i];

        // Filter by RSSI threshold
        if (record->rssi < config.min_rssi_threshold) {
            continue;
        }

        // Convert to our format
        wifi_ap_info_t* ap = &aps[valid_count];
        memcpy(ap->ssid, record->ssid, 32);
        ap->ssid[32] = '\0';

        sprintf(ap->bssid, "%02X:%02X:%02X:%02X:%02X:%02X",
                record->bssid[0], record->bssid[1], record->bssid[2],
                record->bssid[3], record->bssid[4], record->bssid[5]);

        ap->rssi = record->rssi;
        ap->channel = record->primary;
        ap->auth_mode = record->authmode;
        ap->is_hidden = (record->ssid[0] == '\0');

        valid_count++;
    }

    free(ap_records);
    *count = valid_count;

    return ESP_OK;
}

/**
 * @brief Calculate position from WiFi access points
 */
static esp_err_t calculate_position_from_aps(const wifi_ap_info_t* aps, uint8_t ap_count, wifi_position_t* position)
{
    uint64_t start_time = esp_timer_get_time();

    // Find WiGLE entries for each AP
    wigle_entry_t* entries[MAX_VISIBLE_APS];
    int8_t rssis[MAX_VISIBLE_APS];
    uint8_t valid_entries = 0;

    for (uint8_t i = 0; i < ap_count && valid_entries < config.max_ap_count; i++) {
        wigle_entry_t* entry = find_wigle_entry(aps[i].bssid);
        if (entry) {
            entries[valid_entries] = entry;
            rssis[valid_entries] = aps[i].rssi;
            valid_entries++;
            stats.database_hits++;
        }
        stats.database_queries++;
    }

    if (valid_entries < config.min_ap_count) {
        ESP_LOGW(TAG, "Not enough WiGLE entries found: %u (minimum: %u)", valid_entries, config.min_ap_count);
        ESP_LOGW(TAG, "Cannot determine position - insufficient WiGLE database coverage");
        return ESP_ERR_NOT_FOUND;
    }

    ESP_LOGI(TAG, "Found %u matching WiGLE entries", valid_entries);

    // Calculate weighted position
    double accuracy = calculate_distance_weighted_position((const wigle_entry_t**)entries, valid_entries, rssis, position);

    if (accuracy > 1000.0) { // Too inaccurate
        ESP_LOGW(TAG, "Position calculation too inaccurate: %.1fm", accuracy);
        return ESP_ERR_INVALID_RESPONSE;
    }

    position->accuracy_h = (float)accuracy;
    position->accuracy_v = position->accuracy_h * 0.5f; // Estimate vertical accuracy
    position->ap_count = valid_entries;
    position->valid = true;

    // Calculate processing time
    uint64_t end_time = esp_timer_get_time();
    float calc_time = (end_time - start_time) / 1000.0f; // microseconds to milliseconds

    stats.avg_calculation_time_ms = (stats.avg_calculation_time_ms + calc_time) / 2.0f;
    stats.avg_accuracy_m = (stats.avg_accuracy_m + position->accuracy_h) / 2.0f;

    return ESP_OK;
}

/**
 * @brief Find WiGLE entry by BSSID (local database only)
 */
static wigle_entry_t* find_wigle_entry(const char* bssid)
{
    static wigle_entry_t entry;
    wigle_db_entry_t db_entry = {0};  // Initialize to zero

    // Try local WiGLE database first
    if (wigle_db_find_by_bssid(bssid, &db_entry) == ESP_OK) {
        // Copy data from wigle_db_entry_t to wigle_entry_t
        memcpy(&entry, &db_entry, sizeof(wigle_entry_t));
        return &entry;
    }

    // No entry found in local database
    ESP_LOGD(TAG, "No WiGLE entry found for BSSID: %s", bssid);
    return NULL;
}

/**
 * @brief Calculate position using distance-weighted averaging of WiGLE entries
 */
static double calculate_distance_weighted_position(const wigle_entry_t* entries[], uint8_t entry_count,
                                                  const int8_t rssis[], wifi_position_t* position)
{
    if (entry_count == 0) {
        return 999999.0;
    }

    if (entry_count == 1) {
        // Single point
        position->latitude = entries[0]->latitude;
        position->longitude = entries[0]->longitude;
        position->altitude = entries[0]->altitude;
        return entries[0]->accuracy;
    }

    // Calculate weights based on signal strength and distance
    double weights[entry_count];
    double total_weight = 0.0;

    // First pass: calculate distances between points to detect outliers
    double avg_distance = 0.0;
    uint32_t distance_count = 0;

    for (uint8_t i = 0; i < entry_count; i++) {
        for (uint8_t j = i + 1; j < entry_count; j++) {
            double dist = haversine_distance(entries[i]->latitude, entries[i]->longitude,
                                           entries[j]->latitude, entries[j]->longitude);
            avg_distance += dist;
            distance_count++;
        }
    }

    if (distance_count > 0) {
        avg_distance /= distance_count;
    }

    // Second pass: calculate weights, filtering outliers
    uint8_t valid_count = 0;
    for (uint8_t i = 0; i < entry_count; i++) {
        // Convert RSSI to distance estimate (simplified model)
        double rssi_distance = pow(10.0, (-50.0 - rssis[i]) / 20.0) * 1000.0; // Rough estimate in meters

        // Check if this point is an outlier
        bool is_outlier = false;
        if (avg_distance > 0) {
            for (uint8_t j = 0; j < entry_count; j++) {
                if (i != j) {
                    double dist = haversine_distance(entries[i]->latitude, entries[i]->longitude,
                                                   entries[j]->latitude, entries[j]->longitude);
                    if (dist > avg_distance * 3.0) { // 3-sigma outlier detection
                        is_outlier = true;
                        break;
                    }
                }
            }
        }

        if (!is_outlier) {
            // Weight = 1 / (distance + 1) * (RSSI + 100) / 100
            double distance_weight = 1.0 / (rssi_distance + 1.0);
            double rssi_weight = (rssis[i] + 100.0) / 100.0;
            weights[valid_count] = distance_weight * rssi_weight;
            total_weight += weights[valid_count];
            valid_count++;
        }
    }

    if (valid_count == 0) {
        return 999999.0;
    }

    // Calculate weighted average position
    double weighted_lat = 0.0;
    double weighted_lon = 0.0;
    double weighted_alt = 0.0;

    for (uint8_t i = 0; i < valid_count; i++) {
        double normalized_weight = weights[i] / total_weight;
        weighted_lat += entries[i]->latitude * normalized_weight;
        weighted_lon += entries[i]->longitude * normalized_weight;
        weighted_alt += entries[i]->altitude * normalized_weight;
    }

    position->latitude = weighted_lat;
    position->longitude = weighted_lon;
    position->altitude = (float)weighted_alt;

    // Estimate accuracy based on spread of points
    double max_distance = 0.0;
    for (uint8_t i = 0; i < valid_count; i++) {
        double dist = haversine_distance(weighted_lat, weighted_lon,
                                       entries[i]->latitude, entries[i]->longitude);
        if (dist > max_distance) {
            max_distance = dist;
        }
    }

    return max_distance * 1.5; // Conservative accuracy estimate
}

/**
 * @brief Calculate haversine distance between two points
 */
static double haversine_distance(double lat1, double lon1, double lat2, double lon2)
{
    double dlat = (lat2 - lat1) * M_PI / 180.0;
    double dlon = (lon2 - lon1) * M_PI / 180.0;

    double a = sin(dlat/2) * sin(dlat/2) +
               cos(lat1 * M_PI / 180.0) * cos(lat2 * M_PI / 180.0) *
               sin(dlon/2) * sin(dlon/2);

    double c = 2 * atan2(sqrt(a), sqrt(1-a));

    return EARTH_RADIUS_M * c;
}

/**
 * @brief Get current WiFi position estimate
 */
esp_err_t wifi_positioning_get_position(wifi_position_t* position)
{
    if (!initialized || !position) {
        return ESP_ERR_INVALID_ARG;
    }

    *position = last_position;
    return ESP_OK;
}

/**
 * @brief Get WiFi positioning status
 */
wifi_positioning_status_t wifi_positioning_get_status(void)
{
    return status;
}

/**
 * @brief Get WiFi positioning statistics
 */
esp_err_t wifi_positioning_get_stats(wifi_positioning_stats_t* stats_out)
{
    if (!stats_out) {
        return ESP_ERR_INVALID_ARG;
    }

    *stats_out = stats;
    return ESP_OK;
}

/**
 * @brief Check if WiFi positioning is available
 */
bool wifi_positioning_is_available(void)
{
    return initialized && status == WIFI_POS_STATUS_READY;
}

/**
 * @brief Set WiFi positioning configuration
 */
esp_err_t wifi_positioning_set_config(const wifi_positioning_config_t* config_param)
{
    if (!config_param) {
        return ESP_ERR_INVALID_ARG;
    }

    memcpy(&config, config_param, sizeof(wifi_positioning_config_t));
    return ESP_OK;
}

/**
 * @brief Get number of WiFi access points currently visible
 */
uint8_t wifi_positioning_get_visible_ap_count(void)
{
    wifi_ap_info_t aps[MAX_VISIBLE_APS];
    uint8_t count = 0;

    if (perform_wifi_scan(aps, &count, MAX_VISIBLE_APS) != ESP_OK) {
        return 0;
    }

    return count;
}

/**
 * @brief Get list of visible WiFi access points
 */
uint8_t wifi_positioning_get_visible_aps(wifi_ap_info_t* aps, uint8_t max_count)
{
    uint8_t count = 0;

    if (perform_wifi_scan(aps, &count, max_count) != ESP_OK) {
        return 0;
    }

    return count;
}
