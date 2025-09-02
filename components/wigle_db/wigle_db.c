/*
 * WiGLE Database Management Implementation
 * Manages WiGLE WiFi access point database for offline positioning
 */

#include "wigle_db.h"
#include <esp_log.h>
#include <esp_heap_caps.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include "sd_storage.h"

static const char *TAG = "WIGLE_DB";

// Database state
static wigle_db_config_t db_config;
static wigle_db_entry_t* db_entries = NULL;
static uint32_t db_entry_count = 0;
static uint32_t db_capacity = 0;
static bool db_initialized = false;

// Hash table for fast BSSID lookup
#define HASH_TABLE_SIZE 1024
static uint32_t* hash_table = NULL;
static uint32_t hash_table_entries = 0;

// Forward declarations
static uint32_t hash_bssid(const char* bssid);
static int compare_entries(const void* a, const void* b);
static double haversine_distance(double lat1, double lon1, double lat2, double lon2);

/**
 * @brief Initialize WiGLE database system
 */
esp_err_t wigle_db_init(const wigle_db_config_t* config)
{
    if (db_initialized) {
        ESP_LOGW(TAG, "WiGLE database already initialized");
        return ESP_OK;
    }

    if (!config) {
        ESP_LOGE(TAG, "Invalid configuration");
        return ESP_ERR_INVALID_ARG;
    }

    // Copy configuration
    memcpy(&db_config, config, sizeof(wigle_db_config_t));

    // Set defaults
    if (db_config.max_entries == 0) db_config.max_entries = 10000;
    if (db_config.cache_size == 0) db_config.cache_size = 1024;

    // Allocate database memory
    db_capacity = db_config.max_entries;
    db_entries = (wigle_db_entry_t*)heap_caps_malloc(db_capacity * sizeof(wigle_db_entry_t), MALLOC_CAP_SPIRAM);
    if (!db_entries) {
        ESP_LOGE(TAG, "Failed to allocate database memory");
        return ESP_ERR_NO_MEM;
    }

    memset(db_entries, 0, db_capacity * sizeof(wigle_db_entry_t));
    db_entry_count = 0;

    // Allocate hash table
    hash_table = (uint32_t*)heap_caps_malloc(HASH_TABLE_SIZE * sizeof(uint32_t), MALLOC_CAP_SPIRAM);
    if (!hash_table) {
        ESP_LOGE(TAG, "Failed to allocate hash table memory");
        heap_caps_free(db_entries);
        return ESP_ERR_NO_MEM;
    }

    memset(hash_table, 0xFF, HASH_TABLE_SIZE * sizeof(uint32_t)); // Initialize to -1
    hash_table_entries = 0;

    db_initialized = true;

    ESP_LOGI(TAG, "✅ WiGLE database initialized");
    ESP_LOGI(TAG, "Capacity: %u entries, Memory: %u KB",
             db_capacity, (db_capacity * sizeof(wigle_db_entry_t)) / 1024);

    return ESP_OK;
}

/**
 * @brief Deinitialize WiGLE database system
 */
esp_err_t wigle_db_deinit(void)
{
    if (!db_initialized) {
        return ESP_OK;
    }

    // Free memory
    if (db_entries) {
        heap_caps_free(db_entries);
        db_entries = NULL;
    }

    if (hash_table) {
        heap_caps_free(hash_table);
        hash_table = NULL;
    }

    db_entry_count = 0;
    db_capacity = 0;
    db_initialized = false;

    ESP_LOGI(TAG, "✅ WiGLE database deinitialized");

    return ESP_OK;
}

/**
 * @brief Load WiGLE database from file
 */
esp_err_t wigle_db_load_from_file(const char* file_path)
{
    if (!db_initialized) {
        ESP_LOGE(TAG, "Database not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (!file_path) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Loading WiGLE database from: %s", file_path);

    FILE* file = fopen(file_path, "rb");
    if (!file) {
        ESP_LOGE(TAG, "Failed to open database file: %s", file_path);
        return ESP_ERR_NOT_FOUND;
    }

    // Read database header (if any)
    // For now, assume binary format with wigle_db_entry_t records

    uint32_t entries_loaded = 0;
    while (entries_loaded < db_capacity) {
        size_t bytes_read = fread(&db_entries[entries_loaded], sizeof(wigle_db_entry_t), 1, file);
        if (bytes_read != 1) {
            break; // End of file
        }

        // Validate entry
        if (strlen(db_entries[entries_loaded].bssid) > 0 &&
            db_entries[entries_loaded].latitude >= -90.0 && db_entries[entries_loaded].latitude <= 90.0 &&
            db_entries[entries_loaded].longitude >= -180.0 && db_entries[entries_loaded].longitude <= 180.0) {

            // Add to hash table
            uint32_t hash = hash_bssid(db_entries[entries_loaded].bssid);
            hash_table[hash] = entries_loaded;

            entries_loaded++;
        }
    }

    fclose(file);

    db_entry_count = entries_loaded;
    hash_table_entries = entries_loaded;

    ESP_LOGI(TAG, "✅ Loaded %u valid entries from database", db_entry_count);

    return ESP_OK;
}

/**
 * @brief Save WiGLE database to file
 */
esp_err_t wigle_db_save_to_file(const char* file_path)
{
    if (!db_initialized) {
        ESP_LOGE(TAG, "Database not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (!file_path) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Saving WiGLE database to: %s", file_path);

    FILE* file = fopen(file_path, "wb");
    if (!file) {
        ESP_LOGE(TAG, "Failed to create database file: %s", file_path);
        return ESP_ERR_NOT_FOUND;
    }

    // Write all valid entries
    uint32_t entries_written = 0;
    for (uint32_t i = 0; i < db_entry_count; i++) {
        if (strlen(db_entries[i].bssid) > 0) {
            fwrite(&db_entries[i], sizeof(wigle_db_entry_t), 1, file);
            entries_written++;
        }
    }

    fclose(file);

    ESP_LOGI(TAG, "✅ Saved %u entries to database", entries_written);

    return ESP_OK;
}

/**
 * @brief Add entry to WiGLE database
 */
esp_err_t wigle_db_add_entry(const wigle_db_entry_t* entry)
{
    if (!db_initialized) {
        ESP_LOGE(TAG, "Database not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (!entry || strlen(entry->bssid) == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    // Check if entry already exists
    wigle_db_entry_t existing_entry;
    if (wigle_db_find_by_bssid(entry->bssid, &existing_entry) == ESP_OK) {
        // Update existing entry with newer data
        memcpy(&db_entries[hash_table[hash_bssid(entry->bssid)]], entry, sizeof(wigle_db_entry_t));
        return ESP_OK;
    }

    // Check if database is full
    if (db_entry_count >= db_capacity) {
        ESP_LOGW(TAG, "Database full, cannot add new entry");
        return ESP_ERR_NO_MEM;
    }

    // Add new entry
    memcpy(&db_entries[db_entry_count], entry, sizeof(wigle_db_entry_t));

    // Add to hash table
    uint32_t hash = hash_bssid(entry->bssid);
    hash_table[hash] = db_entry_count;

    db_entry_count++;
    hash_table_entries++;

    return ESP_OK;
}

/**
 * @brief Find entry by BSSID
 */
esp_err_t wigle_db_find_by_bssid(const char* bssid, wigle_db_entry_t* entry)
{
    if (!db_initialized || !bssid || !entry) {
        return ESP_ERR_INVALID_ARG;
    }

    uint32_t hash = hash_bssid(bssid);
    uint32_t index = hash_table[hash];

    if (index < db_entry_count && strcmp(db_entries[index].bssid, bssid) == 0) {
        memcpy(entry, &db_entries[index], sizeof(wigle_db_entry_t));
        return ESP_OK;
    }

    return ESP_ERR_NOT_FOUND;
}

/**
 * @brief Find entries within radius of a point
 */
esp_err_t wigle_db_find_nearby(double latitude, double longitude, float radius_m,
                              wigle_db_entry_t* entries, uint32_t max_entries,
                              uint32_t* found_count)
{
    if (!db_initialized || !entries || !found_count) {
        return ESP_ERR_INVALID_ARG;
    }

    *found_count = 0;

    for (uint32_t i = 0; i < db_entry_count && *found_count < max_entries; i++) {
        if (strlen(db_entries[i].bssid) == 0) continue;

        double distance = haversine_distance(latitude, longitude,
                                           db_entries[i].latitude, db_entries[i].longitude);

        if (distance <= radius_m) {
            memcpy(&entries[*found_count], &db_entries[i], sizeof(wigle_db_entry_t));
            (*found_count)++;
        }
    }

    return ESP_OK;
}

/**
 * @brief Get database statistics
 */
esp_err_t wigle_db_get_stats(wigle_db_stats_t* stats)
{
    if (!stats) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(stats, 0, sizeof(wigle_db_stats_t));

    stats->total_entries = db_entry_count;
    stats->valid_entries = 0;
    stats->last_update_timestamp = 0;

    for (uint32_t i = 0; i < db_entry_count; i++) {
        if (strlen(db_entries[i].bssid) > 0) {
            stats->valid_entries++;
            if (db_entries[i].last_update > stats->last_update_timestamp) {
                stats->last_update_timestamp = db_entries[i].last_update;
            }
        }
    }

    stats->database_version = 1;
    strcpy(stats->source_file, db_config.database_path);

    return ESP_OK;
}

/**
 * @brief Clear all entries from database
 */
esp_err_t wigle_db_clear(void)
{
    if (!db_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    memset(db_entries, 0, db_capacity * sizeof(wigle_db_entry_t));
    memset(hash_table, 0xFF, HASH_TABLE_SIZE * sizeof(uint32_t));

    db_entry_count = 0;
    hash_table_entries = 0;

    ESP_LOGI(TAG, "✅ Database cleared");

    return ESP_OK;
}

/**
 * @brief Optimize database
 */
esp_err_t wigle_db_optimize(void)
{
    if (!db_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Optimizing database...");

    // Sort entries by BSSID for better cache performance
    qsort(db_entries, db_entry_count, sizeof(wigle_db_entry_t), compare_entries);

    // Rebuild hash table
    memset(hash_table, 0xFF, HASH_TABLE_SIZE * sizeof(uint32_t));
    hash_table_entries = 0;

    for (uint32_t i = 0; i < db_entry_count; i++) {
        if (strlen(db_entries[i].bssid) > 0) {
            uint32_t hash = hash_bssid(db_entries[i].bssid);
            hash_table[hash] = i;
            hash_table_entries++;
        }
    }

    ESP_LOGI(TAG, "✅ Database optimized");

    return ESP_OK;
}

/**
 * @brief Import from WiGLE CSV format
 * @param csv_path Path to WiGLE CSV file
 * @return ESP_OK on success
 * @note This function is deprecated - convert CSV to binary format on host computer
 */
esp_err_t wigle_db_import_csv(const char* csv_path)
{
    ESP_LOGW(TAG, "CSV import is deprecated. Convert to binary format on host computer.");
    return ESP_ERR_NOT_SUPPORTED;
}

/**
 * @brief Export to WiGLE CSV format
 */
esp_err_t wigle_db_export_csv(const char* csv_path)
{
    if (!db_initialized) {
        ESP_LOGE(TAG, "Database not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (!csv_path) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Exporting to WiGLE CSV: %s", csv_path);

    FILE* file = fopen(csv_path, "w");
    if (!file) {
        ESP_LOGE(TAG, "Failed to create CSV file: %s", csv_path);
        return ESP_ERR_NOT_FOUND;
    }

    // Write CSV header
    fprintf(file, "MAC,SSID,AuthMode,FirstSeen,Channel,RSSI,CurrentLatitude,CurrentLongitude,AltitudeMeters,AccuracyMeters,Type\n");

    // Write entries
    uint32_t entries_exported = 0;
    for (uint32_t i = 0; i < db_entry_count; i++) {
        if (strlen(db_entries[i].bssid) > 0) {
            fprintf(file, "%s,,[WPA2],%lu,6,%d,%.6f,%.6f,%.1f,%u,WIFI\n",
                    db_entries[i].bssid,
                    (unsigned long)db_entries[i].last_update,
                    db_entries[i].avg_rssi,
                    db_entries[i].latitude,
                    db_entries[i].longitude,
                    db_entries[i].altitude,
                    db_entries[i].accuracy);
            entries_exported++;
        }
    }

    fclose(file);

    ESP_LOGI(TAG, "✅ Exported %u entries to CSV", entries_exported);

    return ESP_OK;
}

/**
 * @brief Check if database is loaded and ready
 */
bool wigle_db_is_ready(void)
{
    return db_initialized && db_entry_count > 0;
}

/**
 * @brief Get database memory usage
 */
uint32_t wigle_db_get_memory_usage(void)
{
    if (!db_initialized) {
        return 0;
    }

    uint32_t usage = 0;
    usage += db_capacity * sizeof(wigle_db_entry_t);
    usage += HASH_TABLE_SIZE * sizeof(uint32_t);

    return usage;
}

/**
 * @brief Hash BSSID for fast lookup
 */
static uint32_t hash_bssid(const char* bssid)
{
    uint32_t hash = 0;
    for (int i = 0; bssid[i] != '\0'; i++) {
        hash = hash * 31 + bssid[i];
    }
    return hash % HASH_TABLE_SIZE;
}

/**
 * @brief Compare entries for sorting
 */
static int compare_entries(const void* a, const void* b)
{
    const wigle_db_entry_t* entry_a = (const wigle_db_entry_t*)a;
    const wigle_db_entry_t* entry_b = (const wigle_db_entry_t*)b;

    return strcmp(entry_a->bssid, entry_b->bssid);
}

/**
 * @brief Calculate haversine distance
 */
static double haversine_distance(double lat1, double lon1, double lat2, double lon2)
{
    double dlat = (lat2 - lat1) * M_PI / 180.0;
    double dlon = (lon2 - lon1) * M_PI / 180.0;

    double a = sin(dlat/2) * sin(dlat/2) +
               cos(lat1 * M_PI / 180.0) * cos(lat2 * M_PI / 180.0) *
               sin(dlon/2) * sin(dlon/2);

    double c = 2 * atan2(sqrt(a), sqrt(1-a));

    return 6371000.0 * c; // Earth radius in meters
}
