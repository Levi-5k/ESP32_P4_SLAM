/**
 * @file communication_monitor_c6.h
 * @brief Communication and status monitoring for ESP32-C6
 */

#ifndef COMMUNICATION_MONITOR_C6_H
#define COMMUNICATION_MONITOR_C6_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Monitor configuration
typedef struct {
    uint32_t status_log_interval_ms;    // How often to log status
    uint32_t p4_comm_timeout_ms;        // P4 communication timeout
    bool enable_detailed_logging;       // Enable detailed logging
    bool enable_performance_monitoring; // Enable performance monitoring
} comm_monitor_config_t;

// Communication statistics
typedef struct {
    uint32_t messages_sent_to_p4;       // Messages sent to P4
    uint32_t messages_received_from_p4; // Messages received from P4
    uint32_t web_requests_handled;      // Web requests handled
    uint32_t p4_connection_failures;    // P4 connection failures
    uint32_t last_p4_message_time;      // Last P4 message timestamp
    bool p4_communication_active;       // P4 communication status
    uint32_t uptime_seconds;            // System uptime
    uint32_t free_heap_size;            // Current free heap
    uint32_t min_free_heap_size;        // Minimum free heap recorded
} comm_monitor_stats_t;

/**
 * @brief Initialize communication monitor
 * @param config Monitor configuration
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t comm_monitor_c6_init(const comm_monitor_config_t* config);

/**
 * @brief Start monitoring task
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t comm_monitor_c6_start(void);

/**
 * @brief Stop monitoring task
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t comm_monitor_c6_stop(void);

/**
 * @brief Update P4 communication activity
 * @param message_type Type of message
 * @param is_incoming true if incoming, false if outgoing
 */
void comm_monitor_c6_update_p4_activity(uint8_t message_type, bool is_incoming);

/**
 * @brief Update web activity
 */
void comm_monitor_c6_update_web_activity(void);

/**
 * @brief Get communication statistics
 * @param stats Pointer to store statistics
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t comm_monitor_c6_get_stats(comm_monitor_stats_t* stats);

/**
 * @brief Print comprehensive system status
 */
void comm_monitor_c6_print_comprehensive_status(void);

/**
 * @brief Check if P4 communication is healthy
 * @return true if healthy, false otherwise
 */
bool comm_monitor_c6_is_p4_communication_healthy(void);

#ifdef __cplusplus
}
#endif

#endif // COMMUNICATION_MONITOR_C6_H
