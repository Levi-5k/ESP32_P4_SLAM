/**
 * @file communication_protocol.c
 * @brief Communication protocol implementation for ESP32-C6
 */

#include "communication_protocol.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "COMM_PROTOCOL";

uint16_t comm_calculate_checksum(const void* data, uint16_t size)
{
    if (!data || size == 0) {
        return 0;
    }
    
    const uint8_t* bytes = (const uint8_t*)data;
    uint16_t checksum = 0;
    
    for (uint16_t i = 0; i < size; i++) {
        checksum += bytes[i];
    }
    
    return checksum;
}

bool comm_verify_checksum(const comm_message_t* message)
{
    if (!message) {
        return false;
    }
    
    uint16_t calculated = comm_calculate_checksum(&message->payload, message->header.payload_size);
    return calculated == message->header.checksum;
}
