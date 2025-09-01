#include "sd_storage.h"
#include "esp_vfs_fat.h"
#include "driver/sdmmc_host.h"
#include "sdmmc_cmd.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "sys/stat.h"
#include <string.h>
#include <dirent.h>
#include "driver/gpio.h"
#include "cJSON.h"
#include "visual_slam_types.h"  // For slam_config_t
#include <errno.h>  // For errno
#if SOC_SDMMC_IO_POWER_EXTERNAL
#include "sd_pwr_ctrl_by_on_chip_ldo.h"
#endif

static const char *TAG = "sd_storage_sdmmc";

#define SD_MOUNT_POINT "/sdcard"

// Global state - simplified like ESP-IDF example
static struct {
    bool initialized;
    bool mounted;
    sdmmc_card_t *card;
    sdmmc_host_t host;
    FILE *session_log_file;
    char current_session[32];
} g_sd_state = {0};

// Use the actual SDMMC pins that are wired on the Waveshare ESP32-P4-WIFI6 board
// These pins match the ESP-IDF SDMMC example configuration
static const sd_card_config_t default_sd_config = {
    .clk_pin = GPIO_NUM_43,     // SDMMC CLK pin
    .cmd_pin = GPIO_NUM_44,     // SDMMC CMD pin  
    .d0_pin = GPIO_NUM_39,      // SDMMC D0 pin
    .d1_pin = GPIO_NUM_40,      // SDMMC D1 pin
    .d2_pin = GPIO_NUM_41,      // SDMMC D2 pin  
    .d3_pin = GPIO_NUM_42,      // SDMMC D3 pin
    .max_freq_khz = 20000,      // (20MHz)
    .format_if_mount_failed = false
};

static esp_err_t create_default_directories(void) {
    const char* dirs[] = {
        SD_MOUNT_POINT "/maps",
        SD_MOUNT_POINT "/sessions", 
        SD_MOUNT_POINT "/config",
        SD_MOUNT_POINT "/logs"
    };
    
    for (int i = 0; i < sizeof(dirs) / sizeof(dirs[0]); i++) {
        struct stat st = {0};
        if (stat(dirs[i], &st) == -1) {
            if (mkdir(dirs[i], 0755) != 0) {
                ESP_LOGE(TAG, "Failed to create directory: %s", dirs[i]);
                return ESP_FAIL;
            }
            ESP_LOGI(TAG, "Created directory: %s", dirs[i]);
        }
    }
    return ESP_OK;
}

esp_err_t sd_storage_init(const sd_card_config_t *sd_config) {
    if (g_sd_state.initialized) {
        ESP_LOGW(TAG, "SD storage already initialized");
        return ESP_OK;
    }

    // Use default config if none provided
    if (sd_config == NULL) {
        sd_config = &default_sd_config;
    }

    ESP_LOGI(TAG, "Initializing SD card using SDMMC peripheral on slot 0");
    ESP_LOGI(TAG, "SDMMC pins - CLK:%d, CMD:%d, D0:%d, D1:%d, D2:%d, D3:%d", 
             sd_config->clk_pin, sd_config->cmd_pin, sd_config->d0_pin, 
             sd_config->d1_pin, sd_config->d2_pin, sd_config->d3_pin);

    // Configure GPIO pins for SDMMC (ensure they are set as inputs initially)
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << sd_config->clk_pin) | (1ULL << sd_config->cmd_pin) |
                       (1ULL << sd_config->d0_pin) | (1ULL << sd_config->d1_pin) |
                       (1ULL << sd_config->d2_pin) | (1ULL << sd_config->d3_pin),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    ESP_LOGI(TAG, "SDMMC GPIO pins configured");

    // Mount configuration - following ESP-IDF example exactly
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = sd_config->format_if_mount_failed,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };

    // Initialize SDMMC host - following ESP-IDF example exactly
    g_sd_state.host = (sdmmc_host_t)SDMMC_HOST_DEFAULT();
    g_sd_state.host.slot = SDMMC_HOST_SLOT_0;  // Use slot 0 for the actual SD card pins
    g_sd_state.host.max_freq_khz = sd_config->max_freq_khz;

    // Power control setup for ESP32-P4 (following ESP-IDF example)
#if SOC_SDMMC_IO_POWER_EXTERNAL
    // For ESP32-P4, we need to set up power control for SDMMC IO
    // This matches the ESP-IDF SDMMC example pattern
    sd_pwr_ctrl_ldo_config_t ldo_config = {
        .ldo_chan_id = 4,  // Use LDO channel 4 for SDMMC power (LDO_VO4)
    };
    sd_pwr_ctrl_handle_t pwr_ctrl_handle = NULL;

    esp_err_t ret = sd_pwr_ctrl_new_on_chip_ldo(&ldo_config, &pwr_ctrl_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create SDMMC power control driver: %s", esp_err_to_name(ret));
        return ret;
    }
    g_sd_state.host.pwr_ctrl_handle = pwr_ctrl_handle;
    ESP_LOGI(TAG, "SDMMC power control initialized");
#endif

    // Initialize slot config - following ESP-IDF example exactly
    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    
    // Set our GPIO pins for slot 0
    slot_config.clk = sd_config->clk_pin;
    slot_config.cmd = sd_config->cmd_pin;
    slot_config.d0 = sd_config->d0_pin;
    slot_config.d1 = sd_config->d1_pin;
    slot_config.d2 = sd_config->d2_pin;
    slot_config.d3 = sd_config->d3_pin;
    slot_config.cd = -1;  // No card detect
    slot_config.wp = -1;  // No write protect
    slot_config.width = 4; // 4-bit mode for better performance

    // Mount SD card
    ESP_LOGI(TAG, "Attempting to mount SD card with 4-bit mode at %d kHz", g_sd_state.host.max_freq_khz);

    // Add delay before mounting to ensure SD card is ready
    vTaskDelay(pdMS_TO_TICKS(100));

    esp_err_t ret_mount = esp_vfs_fat_sdmmc_mount(SD_MOUNT_POINT, &g_sd_state.host,
                                            &slot_config, &mount_config, &g_sd_state.card);
    if (ret_mount != ESP_OK) {
        ESP_LOGW(TAG, "4-bit mode failed, trying 1-bit mode as fallback...");

        // Try 1-bit mode as fallback
        slot_config.width = 1;
        g_sd_state.host.max_freq_khz = 1000; // Lower frequency for 1-bit mode

        ret_mount = esp_vfs_fat_sdmmc_mount(SD_MOUNT_POINT, &g_sd_state.host,
                                          &slot_config, &mount_config, &g_sd_state.card);

        if (ret_mount != ESP_OK) {
            ESP_LOGE(TAG, "SD card mount failed with error: %s (0x%x)", esp_err_to_name(ret_mount), ret_mount);
            if (ret_mount == ESP_ERR_TIMEOUT) {
                ESP_LOGE(TAG, "Timeout error - check SD card connections and power");
                ESP_LOGE(TAG, "Troubleshooting tips:");
                ESP_LOGE(TAG, "  1. Ensure SD card is properly inserted");
                ESP_LOGE(TAG, "  2. Check SD card power supply (3.3V)");
                ESP_LOGE(TAG, "  3. Verify pin connections: CLK=%d, CMD=%d, D0=%d, D1=%d, D2=%d, D3=%d",
                         slot_config.clk, slot_config.cmd, slot_config.d0, slot_config.d1, slot_config.d2, slot_config.d3);
                ESP_LOGE(TAG, "  4. Try a different SD card (preferably Class 10 or better)");
                ESP_LOGE(TAG, "  5. Check for voltage level compatibility");
                ESP_LOGE(TAG, "  6. Try running 'sd_storage_diagnose()' for detailed diagnostics");
            } else if (ret_mount == ESP_ERR_INVALID_RESPONSE) {
                ESP_LOGE(TAG, "Invalid response - check SD card compatibility");
                ESP_LOGE(TAG, "  - Try formatting the SD card as FAT32");
                ESP_LOGE(TAG, "  - Ensure SD card is not write-protected");
            } else if (ret_mount == ESP_FAIL) {
                ESP_LOGE(TAG, "General failure - check SD card format and connections");
            }
            return ret_mount;
        } else {
            ESP_LOGW(TAG, "SD card mounted in 1-bit mode (reduced performance)");
        }
    }    ESP_LOGI(TAG, "Filesystem mounted successfully");
    g_sd_state.mounted = true;
    g_sd_state.initialized = true;

    // Card has been initialized, print its properties (ESP-IDF example pattern)
    if (g_sd_state.card) {
        sdmmc_card_print_info(stdout, g_sd_state.card);
        ESP_LOGI(TAG, "SD card mounted at %s", SD_MOUNT_POINT);
    }

    // Create directory structure
    esp_err_t ret_dirs = create_default_directories();
    if (ret_dirs != ESP_OK) {
        ESP_LOGW(TAG, "Failed to create directories: %s", esp_err_to_name(ret_dirs));
    }

    ESP_LOGI(TAG, "SD storage initialization complete");
    return ESP_OK;
}

esp_err_t sd_storage_deinit(void) {
    if (!g_sd_state.initialized) {
        return ESP_OK;
    }

    esp_err_t ret = esp_vfs_fat_sdcard_unmount(SD_MOUNT_POINT, g_sd_state.card);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to unmount SD card: %s", esp_err_to_name(ret));
    }
    
    g_sd_state.initialized = false;
    g_sd_state.mounted = false;
    g_sd_state.card = NULL;
    
    ESP_LOGI(TAG, "SD storage deinitialized");
    return ret;
}

bool sd_storage_is_mounted(void) {
    return g_sd_state.mounted;
}

esp_err_t sd_storage_write_file(const char *filename, const void *data, size_t size) {
    if (!g_sd_state.mounted) {
        ESP_LOGE(TAG, "SD card not mounted");
        return ESP_ERR_INVALID_STATE;
    }

    char filepath[256];
    snprintf(filepath, sizeof(filepath), "%s/%s", SD_MOUNT_POINT, filename);
    ESP_LOGI(TAG, "DEBUG: Attempting to write file: '%s'", filepath);
    ESP_LOGI(TAG, "DEBUG: Filename: '%s', Mount point: '%s'", filename, SD_MOUNT_POINT);

    // Extract directory path and ensure it exists
    char dirpath[256];
    strncpy(dirpath, filepath, sizeof(dirpath) - 1);
    dirpath[sizeof(dirpath) - 1] = '\0';
    
    // Find the last '/' to get directory path
    char *last_slash = strrchr(dirpath, '/');
    if (last_slash != NULL) {
        *last_slash = '\0';  // Terminate string at last slash to get directory
        ESP_LOGI(TAG, "DEBUG: Directory path: '%s'", dirpath);
        
        // Check if directory exists, create if not
        struct stat st = {0};
        if (stat(dirpath, &st) == -1) {
            ESP_LOGI(TAG, "Creating directory recursively: %s", dirpath);
            
            // Create directory recursively
            char temp_path[256];
            char *pos = NULL;
            snprintf(temp_path, sizeof(temp_path), "%s", dirpath);
            
            // Start after mount point
            pos = temp_path + strlen(SD_MOUNT_POINT);
            if (*pos == '/') pos++;
            
            for (char *p = pos; *p; p++) {
                if (*p == '/') {
                    *p = '\0';
                    ESP_LOGD(TAG, "Creating intermediate directory: %s", temp_path);
                    mkdir(temp_path, 0755);  // Ignore errors, might already exist
                    *p = '/';
                }
            }
            
            // Create the final directory
            ESP_LOGD(TAG, "Creating final directory: %s", temp_path);
            if (mkdir(temp_path, 0755) != 0 && errno != EEXIST) {
                ESP_LOGE(TAG, "Failed to create directory %s (errno: %d)", temp_path, errno);
                return ESP_FAIL;
            }
        } else {
            ESP_LOGI(TAG, "DEBUG: Directory already exists: %s", dirpath);
        }
    }

    ESP_LOGI(TAG, "DEBUG: Opening file '%s' for writing", filepath);
    FILE *f = fopen(filepath, "wb");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file %s for writing (errno: %d)", filepath, errno);
        
        // Try with different flags
        ESP_LOGI(TAG, "DEBUG: Trying to open with 'w' flag instead of 'wb'");
        f = fopen(filepath, "w");
        if (f == NULL) {
            ESP_LOGE(TAG, "Also failed with 'w' flag (errno: %d)", errno);
            return ESP_FAIL;
        }
    }

    size_t written = fwrite(data, 1, size, f);
    if (written != size) {
        ESP_LOGE(TAG, "Failed to write complete data to %s (written: %zu, expected: %zu)", filepath, written, size);
        fclose(f);
        return ESP_FAIL;
    }

    fclose(f);
    ESP_LOGI(TAG, "Successfully wrote %zu bytes to %s", size, filepath);
    return ESP_OK;
}

esp_err_t sd_storage_read_file(const char *filename, void **data, size_t *size) {
    if (!g_sd_state.mounted) {
        ESP_LOGE(TAG, "SD card not mounted");
        return ESP_ERR_INVALID_STATE;
    }

    char filepath[256];
    snprintf(filepath, sizeof(filepath), "%s/%s", SD_MOUNT_POINT, filename);

    FILE *f = fopen(filepath, "rb");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file %s for reading", filepath);
        return ESP_FAIL;
    }

    // Get file size
    fseek(f, 0, SEEK_END);
    long file_size = ftell(f);
    fseek(f, 0, SEEK_SET);

    if (file_size <= 0) {
        ESP_LOGE(TAG, "Invalid file size for %s", filepath);
        fclose(f);
        return ESP_FAIL;
    }

    // Allocate memory
    *data = malloc(file_size);
    if (*data == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for file %s", filepath);
        fclose(f);
        return ESP_ERR_NO_MEM;
    }

    size_t read_size = fread(*data, 1, file_size, f);
    fclose(f);

    if (read_size != file_size) {
        ESP_LOGE(TAG, "Failed to read complete file %s", filepath);
        free(*data);
        *data = NULL;
        return ESP_FAIL;
    }

    *size = file_size;
    ESP_LOGI(TAG, "Successfully read %zu bytes from %s", *size, filepath);
    return ESP_OK;
}

esp_err_t sd_storage_list_files(const char *directory, char ***files, size_t *count) {
    if (!g_sd_state.mounted) {
        ESP_LOGE(TAG, "SD card not mounted");
        return ESP_ERR_INVALID_STATE;
    }

    char dirpath[256];
    snprintf(dirpath, sizeof(dirpath), "%s/%s", SD_MOUNT_POINT, directory);

    DIR *dir = opendir(dirpath);
    if (dir == NULL) {
        ESP_LOGE(TAG, "Failed to open directory %s", dirpath);
        return ESP_FAIL;
    }

    // Count files first
    size_t file_count = 0;
    struct dirent *entry;
    while ((entry = readdir(dir)) != NULL) {
        if (entry->d_type == DT_REG) {  // Regular file
            file_count++;
        }
    }
    rewinddir(dir);

    if (file_count == 0) {
        *files = NULL;
        *count = 0;
        closedir(dir);
        return ESP_OK;
    }

    // Allocate array for filenames
    *files = malloc(file_count * sizeof(char*));
    if (*files == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for file list");
        closedir(dir);
        return ESP_ERR_NO_MEM;
    }

    // Read filenames
    size_t index = 0;
    while ((entry = readdir(dir)) != NULL && index < file_count) {
        if (entry->d_type == DT_REG) {
            (*files)[index] = strdup(entry->d_name);
            if ((*files)[index] == NULL) {
                // Clean up on error
                for (size_t i = 0; i < index; i++) {
                    free((*files)[i]);
                }
                free(*files);
                *files = NULL;
                closedir(dir);
                return ESP_ERR_NO_MEM;
            }
            index++;
        }
    }

    closedir(dir);
    *count = file_count;
    ESP_LOGI(TAG, "Found %zu files in %s", file_count, directory);
    return ESP_OK;
}

void sd_storage_free_file_list(char **files, size_t count) {
    if (files == NULL) return;
    
    for (size_t i = 0; i < count; i++) {
        if (files[i] != NULL) {
            free(files[i]);
        }
    }
    free(files);
}

// SLAM-specific wrapper functions
esp_err_t sd_storage_save_slam_map(const char *map_name, double lat, double lon, float alt) {
    ESP_LOGI(TAG, "Saving SLAM map: %s at coords (%.6f, %.6f, %.2f)", map_name, lat, lon, alt);
    // TODO: Implement SLAM map serialization
    return ESP_OK;
}

esp_err_t sd_storage_load_slam_map(const char *map_name, double *lat, double *lon, float *alt) {
    if (!g_sd_state.mounted) {
        ESP_LOGE(TAG, "SD card not mounted");
        return ESP_ERR_INVALID_STATE;
    }

    if (!lat || !lon || !alt) {
        ESP_LOGE(TAG, "Invalid parameters - lat, lon, alt cannot be NULL");
        return ESP_ERR_INVALID_ARG;
    }

    char map_path[128];
    if (map_name) {
        snprintf(map_path, sizeof(map_path), "%s/%s.bin", SD_MAPS_DIR, map_name);
    } else {
        // Try to find the latest map by checking directory
        DIR *dir = opendir(SD_MAPS_DIR);
        if (!dir) {
            ESP_LOGI(TAG, "No maps directory found");
            return ESP_ERR_NOT_FOUND;
        }

        struct dirent *entry;
        char latest_map[64] = {0};
        uint64_t latest_time = 0;

        while ((entry = readdir(dir)) != NULL) {
            if (strstr(entry->d_name, ".bin")) {
                // Check filename length to prevent buffer overflow
                if (strlen(entry->d_name) >= sizeof(latest_map)) {
                    ESP_LOGW(TAG, "Skipping file with long name: %s", entry->d_name);
                    continue;
                }
                
                char full_path[256];  // Increased buffer size
                int written = snprintf(full_path, sizeof(full_path), "%s/%s", SD_MAPS_DIR, entry->d_name);
                if (written >= sizeof(full_path) || written < 0) {
                    ESP_LOGW(TAG, "Skipping file with path too long: %s", entry->d_name);
                    continue;
                }
                
                struct stat st;
                if (stat(full_path, &st) == 0) {
                    if ((uint64_t)st.st_mtime > latest_time) {
                        latest_time = st.st_mtime;
                        strncpy(latest_map, entry->d_name, sizeof(latest_map) - 1);
                        latest_map[sizeof(latest_map) - 1] = '\0';
                        // Remove .bin extension
                        char *dot = strrchr(latest_map, '.');
                        if (dot) *dot = '\0';
                    }
                }
            }
        }
        closedir(dir);

        if (latest_map[0] == '\0') {
            ESP_LOGI(TAG, "No map files found in %s", SD_MAPS_DIR);
            return ESP_ERR_NOT_FOUND;
        }

        snprintf(map_path, sizeof(map_path), "%s/%s.bin", SD_MAPS_DIR, latest_map);
        ESP_LOGI(TAG, "Loading latest map: %s", latest_map);
    }

    // Check if file exists
    FILE *file = fopen(map_path, "rb");
    if (!file) {
        ESP_LOGE(TAG, "Map file not found: %s", map_path);
        return ESP_ERR_NOT_FOUND;
    }

    // Read map header
    map_file_header_t header;
    size_t read_size = fread(&header, 1, sizeof(header), file);
    if (read_size != sizeof(header)) {
        ESP_LOGE(TAG, "Failed to read map header");
        fclose(file);
        return ESP_FAIL;
    }

    // Validate magic number
    if (header.magic != 0x534C414D) { // 'SLAM'
        ESP_LOGE(TAG, "Invalid map file magic number");
        fclose(file);
        return ESP_FAIL;
    }

    // Return GPS origin
    *lat = header.origin_lat;
    *lon = header.origin_lon;
    *alt = header.origin_alt;

    fclose(file);

    ESP_LOGI(TAG, "✅ Map loaded successfully: %s (lat=%.6f, lon=%.6f, alt=%.2f)",
             map_name ? map_name : "latest", *lat, *lon, *alt);
    return ESP_OK;
}

esp_err_t sd_storage_list_maps(char map_names[][64], uint32_t max_maps, uint32_t *count) {
    if (!g_sd_state.mounted) {
        ESP_LOGE(TAG, "SD card not mounted");
        return ESP_ERR_INVALID_STATE;
    }

    if (!map_names || !count) {
        ESP_LOGE(TAG, "Invalid parameters - map_names and count cannot be NULL");
        return ESP_ERR_INVALID_ARG;
    }

    if (max_maps == 0) {
        ESP_LOGE(TAG, "Invalid parameter - max_maps cannot be 0");
        return ESP_ERR_INVALID_ARG;
    }

    DIR *dir = opendir(SD_MAPS_DIR);
    if (!dir) {
        ESP_LOGW(TAG, "Maps directory not found: %s", SD_MAPS_DIR);
        *count = 0;
        return ESP_OK;
    }

    uint32_t map_count = 0;
    struct dirent *entry;

    while ((entry = readdir(dir)) != NULL && map_count < max_maps) {
        if (strstr(entry->d_name, ".bin")) {
            // Remove .bin extension for the map name
            char map_name[64];
            strncpy(map_name, entry->d_name, sizeof(map_name) - 1);
            map_name[sizeof(map_name) - 1] = '\0';

            char *dot = strrchr(map_name, '.');
            if (dot) *dot = '\0';

            strncpy(map_names[map_count], map_name, 63);
            map_names[map_count][63] = '\0';
            map_count++;
        }
    }

    closedir(dir);
    *count = map_count;
    ESP_LOGI(TAG, "Found %u SLAM maps", map_count);
    return ESP_OK;
}

esp_err_t sd_storage_delete_map(const char *map_name) {
    if (!g_sd_state.mounted) {
        ESP_LOGE(TAG, "SD card not mounted");
        return ESP_ERR_INVALID_STATE;
    }

    if (!map_name) {
        ESP_LOGE(TAG, "Map name cannot be NULL");
        return ESP_ERR_INVALID_ARG;
    }

    char filepath[128];
    int written = snprintf(filepath, sizeof(filepath), "%s/%s.bin", SD_MAPS_DIR, map_name);
    if (written >= sizeof(filepath) || written < 0) {
        ESP_LOGE(TAG, "Map filename too long");
        return ESP_ERR_INVALID_ARG;
    }

    if (remove(filepath) != 0) {
        ESP_LOGE(TAG, "Failed to delete map: %s", filepath);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "✅ Map deleted: %s", map_name);
    return ESP_OK;
}

esp_err_t sd_storage_log_slam_pose(uint64_t timestamp, const void *pose) {
    ESP_LOGD(TAG, "Logging SLAM pose at time %llu", timestamp);
    // TODO: Implement pose logging to CSV
    return ESP_OK;
}

esp_err_t sd_storage_start_session_log(const char *session_name) {
    if (!g_sd_state.mounted) {
        ESP_LOGE(TAG, "SD card not mounted");
        return ESP_ERR_INVALID_STATE;
    }

    if (!session_name) {
        ESP_LOGE(TAG, "Session name cannot be NULL");
        return ESP_ERR_INVALID_ARG;
    }

    if (g_sd_state.session_log_file) {
        ESP_LOGW(TAG, "Session log already active, closing previous session");
        fclose(g_sd_state.session_log_file);
        g_sd_state.session_log_file = NULL;
    }

    // Generate session filename with timestamp
    char filename[128];
    uint64_t timestamp = esp_timer_get_time() / 1000000; // Convert to seconds
    int written = snprintf(filename, sizeof(filename), "%s/%s_%llu.csv",
                          SD_SESSIONS_DIR, session_name, timestamp);
    if (written >= sizeof(filename) || written < 0) {
        ESP_LOGE(TAG, "Session filename too long");
        return ESP_ERR_INVALID_ARG;
    }

    g_sd_state.session_log_file = fopen(filename, "w");
    if (!g_sd_state.session_log_file) {
        ESP_LOGE(TAG, "Failed to create session log: %s", filename);
        return ESP_FAIL;
    }

    strncpy(g_sd_state.current_session, session_name, sizeof(g_sd_state.current_session) - 1);
    g_sd_state.current_session[sizeof(g_sd_state.current_session) - 1] = '\0';

    // Write CSV header
    fprintf(g_sd_state.session_log_file,
            "timestamp_us,frame_count,tracked_features,confidence,pos_x,pos_y,pos_z,quat_w,quat_x,quat_y,quat_z\n");

    ESP_LOGI(TAG, "✅ Session log started: %s", filename);
    return ESP_OK;
}

esp_err_t sd_storage_stop_session_log(const session_metadata_t *metadata) {
    if (!g_sd_state.session_log_file) {
        ESP_LOGW(TAG, "No active session log to stop");
        return ESP_ERR_INVALID_STATE;
    }

    fclose(g_sd_state.session_log_file);
    g_sd_state.session_log_file = NULL;

    // Save session metadata as JSON if provided
    if (metadata) {
        char metadata_filename[128];
        int written = snprintf(metadata_filename, sizeof(metadata_filename), "%s/%s_metadata.json",
                              SD_SESSIONS_DIR, g_sd_state.current_session);
        if (written >= sizeof(metadata_filename) || written < 0) {
            ESP_LOGW(TAG, "Metadata filename too long, skipping metadata save");
        } else {
            // TODO: Implement JSON metadata saving when cJSON is available
            ESP_LOGI(TAG, "Session metadata would be saved to: %s", metadata_filename);
        }
    }

    ESP_LOGI(TAG, "✅ Session log stopped: %s", g_sd_state.current_session);
    memset(g_sd_state.current_session, 0, sizeof(g_sd_state.current_session));
    return ESP_OK;
}

esp_err_t sd_storage_get_info(sd_card_info_t *info) {
    if (!g_sd_state.mounted || !info) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Get SD card info from the mounted card
    if (g_sd_state.card) {
        uint64_t total_bytes = (uint64_t)g_sd_state.card->csd.capacity * g_sd_state.card->csd.sector_size;
        info->total_size_mb = total_bytes / (1024 * 1024);
        info->free_size_mb = info->total_size_mb / 2; // Rough estimate
        info->mounted = true;
        strncpy(info->card_name, g_sd_state.card->cid.name, sizeof(info->card_name) - 1);
        info->card_name[sizeof(info->card_name) - 1] = '\0';
        info->speed_class = (g_sd_state.card->real_freq_khz > 25000) ? 10 : 4; // Rough estimate
        ESP_LOGI(TAG, "SD card info: %s, Total=%lluMB, Free~=%lluMB, Class=%u", 
                 info->card_name, info->total_size_mb, info->free_size_mb, info->speed_class);
    } else {
        info->total_size_mb = 0;
        info->free_size_mb = 0;
        info->mounted = false;
        strcpy(info->card_name, "Unknown");
        info->speed_class = 0;
    }
    
    return ESP_OK;
}

// =============================================================================
// CONFIGURATION MANAGEMENT FUNCTIONS
// =============================================================================

esp_err_t sd_storage_save_config(const void *config) {
    if (!g_sd_state.mounted) {
        ESP_LOGE(TAG, "SD card not mounted");
        return ESP_ERR_INVALID_STATE;
    }

    if (!config) {
        ESP_LOGE(TAG, "Config cannot be NULL");
        return ESP_ERR_INVALID_ARG;
    }

    const slam_config_t *slam_config = (const slam_config_t *)config;

    // Ensure config directory exists
    ESP_LOGD(TAG, "Ensuring config directory exists: %s", SD_CONFIG_DIR);
    esp_err_t mkdir_ret = sd_storage_create_directory(SD_CONFIG_DIR);
    if (mkdir_ret != ESP_OK && mkdir_ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Failed to create config directory: %s", esp_err_to_name(mkdir_ret));
        return mkdir_ret;
    }
    ESP_LOGD(TAG, "Config directory ready");

    // Create JSON object
    cJSON *json = cJSON_CreateObject();
    if (!json) {
        ESP_LOGE(TAG, "Failed to create JSON object");
        return ESP_FAIL;
    }

    // Add comprehensive SLAM configuration parameters to JSON
    // ORB Feature Detection Parameters
    cJSON_AddNumberToObject(json, "max_features", slam_config->max_features);
    cJSON_AddNumberToObject(json, "fast_threshold", slam_config->fast_threshold);
    cJSON_AddNumberToObject(json, "levels", slam_config->levels);
    cJSON_AddNumberToObject(json, "scale_factor", slam_config->scale_factor);
    cJSON_AddNumberToObject(json, "max_keypoints_per_level", slam_config->max_keypoints_per_level);
    cJSON_AddBoolToObject(json, "use_harris_detector", slam_config->use_harris_detector);
    cJSON_AddNumberToObject(json, "harris_k", slam_config->harris_k);
    cJSON_AddNumberToObject(json, "descriptor_distance_threshold", slam_config->descriptor_distance_threshold);
    
    // SLAM Algorithm Parameters
    cJSON_AddNumberToObject(json, "match_threshold", slam_config->match_threshold);
    cJSON_AddNumberToObject(json, "min_tracked_features", slam_config->min_tracked_features);
    cJSON_AddNumberToObject(json, "keyframe_distance_threshold", slam_config->keyframe_distance_threshold);
    cJSON_AddNumberToObject(json, "keyframe_angle_threshold", slam_config->keyframe_angle_threshold);
    cJSON_AddNumberToObject(json, "max_keyframes", slam_config->max_keyframes);
    
    // Loop Closure and Optimization
    cJSON_AddBoolToObject(json, "enable_loop_closure", slam_config->enable_loop_closure);
    cJSON_AddNumberToObject(json, "loop_closure_threshold", slam_config->loop_closure_threshold);
    
    // Legacy compatibility fields
    cJSON_AddNumberToObject(json, "orb_features", slam_config->orb_features);
    cJSON_AddNumberToObject(json, "keyframe_threshold", slam_config->keyframe_threshold);
    cJSON_AddBoolToObject(json, "loop_closure_enabled", slam_config->loop_closure_enabled);
    cJSON_AddBoolToObject(json, "map_optimization_enabled", slam_config->map_optimization_enabled);

    // Convert to string
    char *json_string = cJSON_Print(json);
    if (!json_string) {
        ESP_LOGE(TAG, "Failed to convert JSON to string");
        cJSON_Delete(json);
        return ESP_FAIL;
    }

    // Write to file
    esp_err_t ret = sd_storage_write_file("config/slam_config.json", (const uint8_t *)json_string, strlen(json_string));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write config file: %s", esp_err_to_name(ret));
        
        // Additional debugging - check if directory exists
        struct stat st = {0};
        if (stat(SD_CONFIG_DIR, &st) == -1) {
            ESP_LOGE(TAG, "Debug: Config directory does not exist: %s", SD_CONFIG_DIR);
        } else {
            ESP_LOGI(TAG, "Debug: Config directory exists: %s", SD_CONFIG_DIR);
        }
        
        // Try to create the directory one more time
        ESP_LOGI(TAG, "Debug: Attempting to recreate config directory");
        esp_err_t mkdir_debug = sd_storage_create_directory("config");
        ESP_LOGI(TAG, "Debug: Create directory result: %s", esp_err_to_name(mkdir_debug));
        
        free(json_string);
        cJSON_Delete(json);
        return ret;
    }

    ESP_LOGI(TAG, "✅ Config saved to %s", SD_CONFIG_FILE);

    // Cleanup
    free(json_string);
    cJSON_Delete(json);

    return ESP_OK;
}

esp_err_t sd_storage_load_config(void *config) {
    ESP_LOGI(TAG, "🔧 Starting config file loading process...");

    if (!g_sd_state.mounted) {
        ESP_LOGE(TAG, "❌ SD card not mounted - cannot load config");
        return ESP_ERR_INVALID_STATE;
    }

    if (!config) {
        ESP_LOGE(TAG, "❌ Config cannot be NULL");
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "📁 Checking for config file at: %s", SD_CONFIG_FILE);

    // Check if config file exists
    if (!sd_storage_file_exists(SD_CONFIG_FILE)) {
        ESP_LOGW(TAG, "⚠️ No config file found at %s, creating with defaults", SD_CONFIG_FILE);

        // Ensure config directory exists
        ESP_LOGI(TAG, "📁 Creating config directory: %s", SD_CONFIG_DIR);
        esp_err_t mkdir_ret = sd_storage_create_directory(SD_CONFIG_DIR);
        if (mkdir_ret != ESP_OK && mkdir_ret != ESP_ERR_INVALID_STATE) {
            ESP_LOGE(TAG, "❌ Failed to create config directory: %s", esp_err_to_name(mkdir_ret));
            return mkdir_ret;
        }
        ESP_LOGI(TAG, "✅ Config directory ready");

        // Create comprehensive default SLAM configuration
        slam_config_t default_config = {
            // ORB Feature Detection Parameters
            .max_features = 500,                    // Maximum number of features to extract
            .fast_threshold = 20.0f,               // FAST corner detection threshold
            .levels = 8,                           // Number of pyramid levels
            .scale_factor = 1.2f,                  // Scale factor between levels
            .max_keypoints_per_level = 100,        // Max keypoints per pyramid level
            .use_harris_detector = false,          // Use Harris corner detector
            .harris_k = 0.04f,                     // Harris detector k parameter
            .descriptor_distance_threshold = 50,    // Descriptor matching distance threshold
            
            // SLAM Algorithm Parameters
            .match_threshold = 0.7f,               // Feature matching threshold
            .min_tracked_features = 30,            // Minimum features for tracking
            .keyframe_distance_threshold = 1.0f,   // Keyframe insertion distance threshold
            .keyframe_angle_threshold = 0.2f,      // Keyframe insertion angle threshold
            .max_keyframes = 50,                   // Maximum number of keyframes
            
            // Loop Closure and Optimization
            .enable_loop_closure = true,           // Enable loop closure detection
            .loop_closure_threshold = 0.8f,        // Loop closure detection threshold
            
            // Legacy compatibility fields
            .orb_features = 500,                   // Number of ORB features to extract
            .keyframe_threshold = 15,              // Frames between keyframes
            .loop_closure_enabled = true,          // Enable loop closure detection
            .map_optimization_enabled = true       // Enable map optimization
        };

        ESP_LOGI(TAG, "💾 Saving default config to file...");
        // Save default config to file
        esp_err_t save_ret = sd_storage_save_config(&default_config);
        if (save_ret != ESP_OK) {
            ESP_LOGE(TAG, "❌ Failed to save default config: %s", esp_err_to_name(save_ret));
            return save_ret;
        }

        ESP_LOGI(TAG, "✅ Created default config file at %s", SD_CONFIG_FILE);
        ESP_LOGI(TAG, "  ORB Features: max=%u, threshold=%.1f, levels=%u, scale=%.1f", 
                 default_config.max_features, default_config.fast_threshold,
                 default_config.levels, default_config.scale_factor);
        ESP_LOGI(TAG, "  SLAM Params: min_features=%u, keyframes=%u, match_thresh=%.1f",
                 default_config.min_tracked_features, default_config.max_keyframes, default_config.match_threshold);
        ESP_LOGI(TAG, "  Loop closure: %s (thresh=%.1f), Map optimization: %s",
                 default_config.enable_loop_closure ? "enabled" : "disabled",
                 default_config.loop_closure_threshold,
                 default_config.map_optimization_enabled ? "enabled" : "disabled");

        // Copy default config to output
        memcpy(config, &default_config, sizeof(slam_config_t));
        return ESP_OK;
    }

    // Config file exists, load it
    ESP_LOGI(TAG, "📖 Loading existing config from %s", SD_CONFIG_FILE);

    // Read the config file
    void *file_data = NULL;
    size_t file_size = 0;
    esp_err_t read_ret = sd_storage_read_file("config/slam_config.json", &file_data, &file_size);
    if (read_ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read config file: %s", esp_err_to_name(read_ret));

        // Fall back to comprehensive defaults
        slam_config_t default_config = {
            // ORB Feature Detection Parameters
            .max_features = 500,
            .fast_threshold = 20.0f,
            .levels = 8,
            .scale_factor = 1.2f,
            .max_keypoints_per_level = 100,
            .use_harris_detector = false,
            .harris_k = 0.04f,
            .descriptor_distance_threshold = 50,
            
            // SLAM Algorithm Parameters
            .match_threshold = 0.7f,
            .min_tracked_features = 30,
            .keyframe_distance_threshold = 1.0f,
            .keyframe_angle_threshold = 0.2f,
            .max_keyframes = 50,
            
            // Loop Closure and Optimization
            .enable_loop_closure = true,
            .loop_closure_threshold = 0.8f,
            
            // Legacy compatibility fields
            .orb_features = 500,
            .keyframe_threshold = 30,
            .loop_closure_enabled = true,
            .map_optimization_enabled = true
        };

        memcpy(config, &default_config, sizeof(slam_config_t));
        return ESP_OK;
    }

    // Parse JSON
    cJSON *json = cJSON_Parse((const char *)file_data);
    if (!json) {
        ESP_LOGE(TAG, "Failed to parse JSON config file");
        free(file_data);

        // Fall back to comprehensive defaults
        slam_config_t default_config = {
            // ORB Feature Detection Parameters
            .max_features = 500,
            .fast_threshold = 20.0f,
            .levels = 8,
            .scale_factor = 1.2f,
            .max_keypoints_per_level = 100,
            .use_harris_detector = false,
            .harris_k = 0.04f,
            .descriptor_distance_threshold = 50,
            
            // SLAM Algorithm Parameters
            .match_threshold = 0.7f,
            .min_tracked_features = 30,
            .keyframe_distance_threshold = 1.0f,
            .keyframe_angle_threshold = 0.2f,
            .max_keyframes = 50,
            
            // Loop Closure and Optimization
            .enable_loop_closure = true,
            .loop_closure_threshold = 0.8f,
            
            // Legacy compatibility fields
            .orb_features = 500,
            .keyframe_threshold = 30,
            .loop_closure_enabled = true,
            .map_optimization_enabled = true
        };

        memcpy(config, &default_config, sizeof(slam_config_t));
        return ESP_OK;
    }

    // Extract values from JSON with comprehensive defaults
    slam_config_t loaded_config = {0};

    // Load ORB Feature Detection Parameters with defaults
    loaded_config.max_features = cJSON_GetObjectItem(json, "max_features") ?
        cJSON_GetObjectItem(json, "max_features")->valueint : 500;
    loaded_config.fast_threshold = cJSON_GetObjectItem(json, "fast_threshold") ?
        (float)cJSON_GetObjectItem(json, "fast_threshold")->valuedouble : 20.0f;
    loaded_config.levels = cJSON_GetObjectItem(json, "levels") ?
        cJSON_GetObjectItem(json, "levels")->valueint : 8;
    loaded_config.scale_factor = cJSON_GetObjectItem(json, "scale_factor") ?
        (float)cJSON_GetObjectItem(json, "scale_factor")->valuedouble : 1.2f;
    loaded_config.max_keypoints_per_level = cJSON_GetObjectItem(json, "max_keypoints_per_level") ?
        cJSON_GetObjectItem(json, "max_keypoints_per_level")->valueint : 100;
    loaded_config.use_harris_detector = cJSON_GetObjectItem(json, "use_harris_detector") ?
        cJSON_IsTrue(cJSON_GetObjectItem(json, "use_harris_detector")) : false;
    loaded_config.harris_k = cJSON_GetObjectItem(json, "harris_k") ?
        (float)cJSON_GetObjectItem(json, "harris_k")->valuedouble : 0.04f;
    loaded_config.descriptor_distance_threshold = cJSON_GetObjectItem(json, "descriptor_distance_threshold") ?
        cJSON_GetObjectItem(json, "descriptor_distance_threshold")->valueint : 50;

    // Load SLAM Algorithm Parameters with defaults
    loaded_config.match_threshold = cJSON_GetObjectItem(json, "match_threshold") ?
        (float)cJSON_GetObjectItem(json, "match_threshold")->valuedouble : 0.7f;
    loaded_config.min_tracked_features = cJSON_GetObjectItem(json, "min_tracked_features") ?
        cJSON_GetObjectItem(json, "min_tracked_features")->valueint : 30;
    loaded_config.keyframe_distance_threshold = cJSON_GetObjectItem(json, "keyframe_distance_threshold") ?
        (float)cJSON_GetObjectItem(json, "keyframe_distance_threshold")->valuedouble : 1.0f;
    loaded_config.keyframe_angle_threshold = cJSON_GetObjectItem(json, "keyframe_angle_threshold") ?
        (float)cJSON_GetObjectItem(json, "keyframe_angle_threshold")->valuedouble : 0.2f;
    loaded_config.max_keyframes = cJSON_GetObjectItem(json, "max_keyframes") ?
        cJSON_GetObjectItem(json, "max_keyframes")->valueint : 50;

    // Load Loop Closure and Optimization Parameters with defaults
    loaded_config.enable_loop_closure = cJSON_GetObjectItem(json, "enable_loop_closure") ?
        cJSON_IsTrue(cJSON_GetObjectItem(json, "enable_loop_closure")) : true;
    loaded_config.loop_closure_threshold = cJSON_GetObjectItem(json, "loop_closure_threshold") ?
        (float)cJSON_GetObjectItem(json, "loop_closure_threshold")->valuedouble : 0.8f;

    // Load Legacy compatibility fields with defaults (for backward compatibility)
    loaded_config.orb_features = cJSON_GetObjectItem(json, "orb_features") ?
        cJSON_GetObjectItem(json, "orb_features")->valueint : loaded_config.max_features;
    loaded_config.keyframe_threshold = cJSON_GetObjectItem(json, "keyframe_threshold") ?
        cJSON_GetObjectItem(json, "keyframe_threshold")->valueint : 15;
    loaded_config.loop_closure_enabled = cJSON_GetObjectItem(json, "loop_closure_enabled") ?
        cJSON_IsTrue(cJSON_GetObjectItem(json, "loop_closure_enabled")) : loaded_config.enable_loop_closure;
    loaded_config.map_optimization_enabled = cJSON_GetObjectItem(json, "map_optimization_enabled") ?
        cJSON_IsTrue(cJSON_GetObjectItem(json, "map_optimization_enabled")) : true;

    // Copy to output
    memcpy(config, &loaded_config, sizeof(slam_config_t));

    ESP_LOGI(TAG, "✅ Config loaded successfully from %s", SD_CONFIG_FILE);
    ESP_LOGI(TAG, "  ORB Features: max=%u, threshold=%.1f, levels=%u, scale=%.1f", 
             loaded_config.max_features, loaded_config.fast_threshold,
             loaded_config.levels, loaded_config.scale_factor);
    ESP_LOGI(TAG, "  SLAM Params: min_features=%u, keyframes=%u, match_thresh=%.1f",
             loaded_config.min_tracked_features, loaded_config.max_keyframes, loaded_config.match_threshold);
    ESP_LOGI(TAG, "  Loop closure: %s (thresh=%.1f), Map optimization: %s",
             loaded_config.enable_loop_closure ? "enabled" : "disabled",
             loaded_config.loop_closure_threshold,
             loaded_config.map_optimization_enabled ? "enabled" : "disabled");

    // Check if the loaded configuration is missing any parameters and re-save if needed
    bool needs_resave = false;
    if (!cJSON_GetObjectItem(json, "max_features") ||
        !cJSON_GetObjectItem(json, "fast_threshold") ||
        !cJSON_GetObjectItem(json, "levels") ||
        !cJSON_GetObjectItem(json, "scale_factor") ||
        !cJSON_GetObjectItem(json, "max_keypoints_per_level") ||
        !cJSON_GetObjectItem(json, "use_harris_detector") ||
        !cJSON_GetObjectItem(json, "harris_k") ||
        !cJSON_GetObjectItem(json, "descriptor_distance_threshold") ||
        !cJSON_GetObjectItem(json, "match_threshold") ||
        !cJSON_GetObjectItem(json, "min_tracked_features") ||
        !cJSON_GetObjectItem(json, "keyframe_distance_threshold") ||
        !cJSON_GetObjectItem(json, "keyframe_angle_threshold") ||
        !cJSON_GetObjectItem(json, "max_keyframes") ||
        !cJSON_GetObjectItem(json, "enable_loop_closure") ||
        !cJSON_GetObjectItem(json, "loop_closure_threshold") ||
        !cJSON_GetObjectItem(json, "orb_features") ||
        !cJSON_GetObjectItem(json, "keyframe_threshold") ||
        !cJSON_GetObjectItem(json, "loop_closure_enabled") ||
        !cJSON_GetObjectItem(json, "map_optimization_enabled")) {
        needs_resave = true;
    }

    if (needs_resave) {
        ESP_LOGI(TAG, "💾 Configuration missing parameters, updating file with missing defaults...");
        
        // Add only missing parameters to existing JSON (preserving custom values)
        if (!cJSON_GetObjectItem(json, "max_features"))
            cJSON_AddNumberToObject(json, "max_features", loaded_config.max_features);
        if (!cJSON_GetObjectItem(json, "fast_threshold"))
            cJSON_AddNumberToObject(json, "fast_threshold", loaded_config.fast_threshold);
        if (!cJSON_GetObjectItem(json, "levels"))
            cJSON_AddNumberToObject(json, "levels", loaded_config.levels);
        if (!cJSON_GetObjectItem(json, "scale_factor"))
            cJSON_AddNumberToObject(json, "scale_factor", loaded_config.scale_factor);
        if (!cJSON_GetObjectItem(json, "max_keypoints_per_level"))
            cJSON_AddNumberToObject(json, "max_keypoints_per_level", loaded_config.max_keypoints_per_level);
        if (!cJSON_GetObjectItem(json, "use_harris_detector"))
            cJSON_AddBoolToObject(json, "use_harris_detector", loaded_config.use_harris_detector);
        if (!cJSON_GetObjectItem(json, "harris_k"))
            cJSON_AddNumberToObject(json, "harris_k", loaded_config.harris_k);
        if (!cJSON_GetObjectItem(json, "descriptor_distance_threshold"))
            cJSON_AddNumberToObject(json, "descriptor_distance_threshold", loaded_config.descriptor_distance_threshold);
        if (!cJSON_GetObjectItem(json, "match_threshold"))
            cJSON_AddNumberToObject(json, "match_threshold", loaded_config.match_threshold);
        if (!cJSON_GetObjectItem(json, "min_tracked_features"))
            cJSON_AddNumberToObject(json, "min_tracked_features", loaded_config.min_tracked_features);
        if (!cJSON_GetObjectItem(json, "keyframe_distance_threshold"))
            cJSON_AddNumberToObject(json, "keyframe_distance_threshold", loaded_config.keyframe_distance_threshold);
        if (!cJSON_GetObjectItem(json, "keyframe_angle_threshold"))
            cJSON_AddNumberToObject(json, "keyframe_angle_threshold", loaded_config.keyframe_angle_threshold);
        if (!cJSON_GetObjectItem(json, "max_keyframes"))
            cJSON_AddNumberToObject(json, "max_keyframes", loaded_config.max_keyframes);
        if (!cJSON_GetObjectItem(json, "enable_loop_closure"))
            cJSON_AddBoolToObject(json, "enable_loop_closure", loaded_config.enable_loop_closure);
        if (!cJSON_GetObjectItem(json, "loop_closure_threshold"))
            cJSON_AddNumberToObject(json, "loop_closure_threshold", loaded_config.loop_closure_threshold);
        if (!cJSON_GetObjectItem(json, "orb_features"))
            cJSON_AddNumberToObject(json, "orb_features", loaded_config.orb_features);
        if (!cJSON_GetObjectItem(json, "keyframe_threshold"))
            cJSON_AddNumberToObject(json, "keyframe_threshold", loaded_config.keyframe_threshold);
        if (!cJSON_GetObjectItem(json, "loop_closure_enabled"))
            cJSON_AddBoolToObject(json, "loop_closure_enabled", loaded_config.loop_closure_enabled);
        if (!cJSON_GetObjectItem(json, "map_optimization_enabled"))
            cJSON_AddBoolToObject(json, "map_optimization_enabled", loaded_config.map_optimization_enabled);

        // Write the updated JSON back to file
        char *updated_json_string = cJSON_Print(json);
        if (updated_json_string) {
            FILE *file = fopen(SD_CONFIG_FILE, "w");
            if (file) {
                size_t written = fwrite(updated_json_string, 1, strlen(updated_json_string), file);
                fclose(file);
                if (written == strlen(updated_json_string)) {
                    ESP_LOGI(TAG, "✅ Configuration file updated with missing parameters only");
                } else {
                    ESP_LOGW(TAG, "⚠️ Partial write to configuration file");
                }
            } else {
                ESP_LOGW(TAG, "⚠️ Failed to open configuration file for writing");
            }
            free(updated_json_string);
        } else {
            ESP_LOGW(TAG, "⚠️ Failed to serialize updated JSON");
        }
    }

    // Cleanup
    cJSON_Delete(json);
    free(file_data);

    return ESP_OK;
}

esp_err_t sd_storage_get_file_size(const char *filepath, size_t *size) {
    if (!g_sd_state.mounted) {
        ESP_LOGE(TAG, "SD card not mounted");
        return ESP_ERR_INVALID_STATE;
    }

    if (!filepath || !size) {
        ESP_LOGE(TAG, "Invalid parameters - filepath and size cannot be NULL");
        return ESP_ERR_INVALID_ARG;
    }

    struct stat st;
    if (stat(filepath, &st) != 0) {
        ESP_LOGE(TAG, "Failed to get file stats for: %s", filepath);
        return ESP_FAIL;
    }

    *size = st.st_size;
    ESP_LOGD(TAG, "File size: %s = %zu bytes", filepath, *size);
    return ESP_OK;
}

esp_err_t sd_storage_diagnose(void) {
    ESP_LOGI(TAG, "=== SD Card Diagnostic Report ===");

    // Check if already initialized
    if (g_sd_state.mounted) {
        ESP_LOGI(TAG, "✅ SD card is already mounted at %s", SD_MOUNT_POINT);
        return ESP_OK;
    }

    // Test GPIO pin configuration
    ESP_LOGI(TAG, "Testing GPIO pin configuration...");
    const sd_card_config_t *config = &default_sd_config;

    // Check if pins are accessible
    for (int i = 0; i < 6; i++) {
        gpio_num_t pin;
        const char *pin_name;

        switch (i) {
            case 0: pin = config->clk_pin; pin_name = "CLK"; break;
            case 1: pin = config->cmd_pin; pin_name = "CMD"; break;
            case 2: pin = config->d0_pin; pin_name = "D0"; break;
            case 3: pin = config->d1_pin; pin_name = "D1"; break;
            case 4: pin = config->d2_pin; pin_name = "D2"; break;
            case 5: pin = config->d3_pin; pin_name = "D3"; break;
        }

        // Try to configure pin as input to test accessibility
        gpio_config_t test_conf = {
            .pin_bit_mask = (1ULL << pin),
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE
        };

        esp_err_t gpio_ret = gpio_config(&test_conf);
        if (gpio_ret == ESP_OK) {
            ESP_LOGI(TAG, "  ✅ GPIO %d (%s) is accessible", pin, pin_name);
        } else {
            ESP_LOGE(TAG, "  ❌ GPIO %d (%s) configuration failed: %s", pin, pin_name, esp_err_to_name(gpio_ret));
        }
    }

    // Test SDMMC host initialization
    ESP_LOGI(TAG, "Testing SDMMC host initialization...");
    sdmmc_host_t test_host = (sdmmc_host_t)SDMMC_HOST_DEFAULT();
    test_host.slot = SDMMC_HOST_SLOT_0;
    test_host.max_freq_khz = 1000; // Start with low frequency for testing

    // Test slot configuration
    sdmmc_slot_config_t test_slot = SDMMC_SLOT_CONFIG_DEFAULT();
    test_slot.clk = config->clk_pin;
    test_slot.cmd = config->cmd_pin;
    test_slot.d0 = config->d0_pin;
    test_slot.d1 = config->d1_pin;
    test_slot.d2 = config->d2_pin;
    test_slot.d3 = config->d3_pin;
    test_slot.cd = -1;
    test_slot.wp = -1;
    test_slot.width = 1; // Start with 1-bit mode for testing

    ESP_LOGI(TAG, "Testing SD card detection with 1-bit mode at 1MHz...");

    // Try to initialize card with minimal settings
    sdmmc_card_t *test_card = NULL;
    esp_vfs_fat_sdmmc_mount_config_t test_mount = {
        .format_if_mount_failed = false,
        .max_files = 1,
        .allocation_unit_size = 16 * 1024
    };

    esp_err_t test_ret = esp_vfs_fat_sdmmc_mount(SD_MOUNT_POINT "_test", &test_host,
                                                &test_slot, &test_mount, &test_card);

    if (test_ret == ESP_OK) {
        ESP_LOGI(TAG, "✅ SD card detected and mounted successfully!");
        ESP_LOGI(TAG, "  Card info:");
        if (test_card) {
            sdmmc_card_print_info(stdout, test_card);
        }

        // Unmount test mount
        esp_vfs_fat_sdcard_unmount(SD_MOUNT_POINT "_test", test_card);
        ESP_LOGI(TAG, "Test mount unmounted");
    } else {
        ESP_LOGE(TAG, "❌ SD card test failed: %s (0x%x)", esp_err_to_name(test_ret), test_ret);

        // Provide specific troubleshooting based on error
        switch (test_ret) {
            case ESP_ERR_TIMEOUT:
                ESP_LOGI(TAG, "TIMEOUT ERROR - Most likely causes:");
                ESP_LOGI(TAG, "  • SD card not inserted or loose connection");
                ESP_LOGI(TAG, "  • Insufficient power to SD card (needs 3.3V)");
                ESP_LOGI(TAG, "  • Incorrect pin wiring");
                ESP_LOGI(TAG, "  • SD card is write-protected");
                break;
            case ESP_ERR_INVALID_RESPONSE:
                ESP_LOGI(TAG, "INVALID RESPONSE - Possible causes:");
                ESP_LOGI(TAG, "  • SD card not compatible with SDMMC interface");
                ESP_LOGI(TAG, "  • SD card is corrupted or damaged");
                ESP_LOGI(TAG, "  • Wrong voltage level (check for 3.3V compatibility)");
                break;
            default:
                ESP_LOGI(TAG, "GENERAL ERROR - Check:");
                ESP_LOGI(TAG, "  • SD card format (should be FAT32)");
                ESP_LOGI(TAG, "  • All SDMMC pins properly connected");
                ESP_LOGI(TAG, "  • No short circuits on SD card pins");
        }
    }

    ESP_LOGI(TAG, "=== End SD Card Diagnostic ===");
    return test_ret;
}

// ============================================================================
// FILE SYSTEM UTILITIES
// ============================================================================

/**
 * Check if a file exists on the SD card
 */
bool sd_storage_file_exists(const char *filepath) {
    if (!g_sd_state.mounted) {
        ESP_LOGE(TAG, "SD card not mounted - cannot check file existence");
        return false;
    }

    if (!filepath) {
        ESP_LOGE(TAG, "Filepath cannot be NULL");
        return false;
    }

    // Use stat to check if file exists
    struct stat st;
    int ret = stat(filepath, &st);

    if (ret == 0) {
        ESP_LOGD(TAG, "File exists: %s", filepath);
        return true;
    } else {
        ESP_LOGD(TAG, "File does not exist: %s (errno: %d)", filepath, errno);
        return false;
    }
}

/**
 * Create a directory on the SD card (recursive)
 */
esp_err_t sd_storage_create_directory(const char *dirpath) {
    if (!g_sd_state.mounted) {
        ESP_LOGE(TAG, "SD card not mounted - cannot create directory");
        return ESP_ERR_INVALID_STATE;
    }

    if (!dirpath) {
        ESP_LOGE(TAG, "Directory path cannot be NULL");
        return ESP_ERR_INVALID_ARG;
    }

    char fullpath[256];

    // Check if dirpath already starts with mount point
    if (strncmp(dirpath, SD_MOUNT_POINT, strlen(SD_MOUNT_POINT)) == 0) {
        // Already an absolute path, use as-is
        ESP_LOGD(TAG, "Using absolute path: %s", dirpath);
        strncpy(fullpath, dirpath, sizeof(fullpath) - 1);
        fullpath[sizeof(fullpath) - 1] = '\0';
    } else {
        // Relative path, add mount point
        ESP_LOGD(TAG, "Converting relative path '%s' to absolute", dirpath);
        snprintf(fullpath, sizeof(fullpath), "%s/%s", SD_MOUNT_POINT, dirpath);
    }

    ESP_LOGD(TAG, "Creating directory: %s", fullpath);

    // Check if directory already exists
    struct stat st = {0};
    if (stat(fullpath, &st) == 0) {
        if (S_ISDIR(st.st_mode)) {
            ESP_LOGD(TAG, "Directory already exists: %s", fullpath);
            return ESP_OK;
        } else {
            ESP_LOGE(TAG, "Path exists but is not a directory: %s", fullpath);
            return ESP_FAIL;
        }
    }

    // Create directory (mkdir only creates one level)
    int ret = mkdir(fullpath, 0755);
    if (ret == 0) {
        ESP_LOGD(TAG, "Directory created successfully: %s", fullpath);
        return ESP_OK;
    } else {
        if (errno == EEXIST) {
            ESP_LOGD(TAG, "Directory already exists: %s", fullpath);
            return ESP_OK;  // Directory already exists, not an error
        } else if (errno == ENOENT) {
            // Parent directory doesn't exist, create it first
            char parent[256];
            strncpy(parent, fullpath, sizeof(parent) - 1);
            parent[sizeof(parent) - 1] = '\0';
            
            char *last_slash = strrchr(parent, '/');
            if (last_slash != NULL && last_slash != parent) {
                *last_slash = '\0';  // Remove last component
                ESP_LOGD(TAG, "Creating parent directory: %s", parent);
                
                esp_err_t parent_ret = sd_storage_create_directory(parent);
                if (parent_ret != ESP_OK) {
                    ESP_LOGE(TAG, "Failed to create parent directory: %s", parent);
                    return parent_ret;
                }
                
                // Try creating the original directory again
                ret = mkdir(fullpath, 0755);
                if (ret == 0) {
                    ESP_LOGD(TAG, "Directory created successfully: %s", fullpath);
                    return ESP_OK;
                }
            }
        }
        
        ESP_LOGE(TAG, "Failed to create directory %s: errno %d", fullpath, errno);
        return ESP_FAIL;
    }
}
