#include "sd_storage.h"
#include "esp_vfs_fat.h"
#include "driver/sdmmc_host.h"
#include "sdmmc_cmd.h"
#include "esp_log.h"
#include "sys/stat.h"
#include <string.h>
#include <dirent.h>
#include "driver/gpio.h"
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
    esp_err_t ret_mount = esp_vfs_fat_sdmmc_mount(SD_MOUNT_POINT, &g_sd_state.host, 
                                            &slot_config, &mount_config, &g_sd_state.card);
    if (ret_mount != ESP_OK) {
        ESP_LOGE(TAG, "SD card mount failed with error: %s (0x%x)", esp_err_to_name(ret_mount), ret_mount);
        if (ret_mount == ESP_ERR_TIMEOUT) {
            ESP_LOGE(TAG, "Timeout error - check SD card connections and power");
        } else if (ret_mount == ESP_ERR_INVALID_RESPONSE) {
            ESP_LOGE(TAG, "Invalid response - check SD card compatibility");
        } else if (ret_mount == ESP_FAIL) {
            ESP_LOGE(TAG, "General failure - check SD card format and connections");
        }
        return ret_mount;
    }
    
    ESP_LOGI(TAG, "Filesystem mounted successfully");
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

    FILE *f = fopen(filepath, "wb");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file %s for writing", filepath);
        return ESP_FAIL;
    }

    size_t written = fwrite(data, 1, size, f);
    fclose(f);

    if (written != size) {
        ESP_LOGE(TAG, "Failed to write complete data to %s", filepath);
        return ESP_FAIL;
    }

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

    ESP_LOGI(TAG, "Loading SLAM map: %s", map_name ? map_name : "latest");
    // TODO: Implement SLAM map deserialization
    *lat = 0.0;
    *lon = 0.0;
    *alt = 0.0;
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

    ESP_LOGI(TAG, "Listing available SLAM maps");
    // TODO: Implement proper map listing from /sdcard/maps directory
    *count = 0;
    return ESP_OK;
}

esp_err_t sd_storage_log_slam_pose(uint64_t timestamp, const void *pose) {
    ESP_LOGD(TAG, "Logging SLAM pose at time %llu", timestamp);
    // TODO: Implement pose logging to CSV
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
