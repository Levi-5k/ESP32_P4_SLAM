/**
 * SDMMC Arbitrator
 * Manages sequential access to SDMMC peripheral between SD card and ESP-Hosted WiFi
 * 
 * This component addresses the known ESP-IDF issue where SDMMC and ESP-Hosted SDIO
 * cannot operate simultaneously due to sd_host_claim_controller conflicts.
 */

#ifndef SDMMC_ARBITRATOR_H
#define SDMMC_ARBITRATOR_H

#include "esp_err.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    SDMMC_USER_SD_CARD = 0,
    SDMMC_USER_ESP_HOSTED,
    SDMMC_USER_NONE
} sdmmc_user_t;

/**
 * Initialize the SDMMC arbitrator
 */
esp_err_t sdmmc_arbitrator_init(void);

/**
 * Request access to SDMMC peripheral
 * @param user The requesting user (SD card or ESP-Hosted)
 * @param timeout_ms Timeout in milliseconds to wait for access
 * @return ESP_OK on success, ESP_ERR_TIMEOUT if timeout occurred
 */
esp_err_t sdmmc_arbitrator_request_access(sdmmc_user_t user, uint32_t timeout_ms);

/**
 * Release access to SDMMC peripheral
 * @param user The user releasing access
 * @return ESP_OK on success
 */
esp_err_t sdmmc_arbitrator_release_access(sdmmc_user_t user);

/**
 * Check if SDMMC is currently in use
 * @return Current user or SDMMC_USER_NONE if available
 */
sdmmc_user_t sdmmc_arbitrator_get_current_user(void);

/**
 * Force release access (emergency use only)
 */
esp_err_t sdmmc_arbitrator_force_release(void);

#ifdef __cplusplus
}
#endif

#endif // SDMMC_ARBITRATOR_H
