idf_component_register(
    SRCS "communication_manager.c"
         "communication_protocol.c"
    INCLUDE_DIRS "include"
    REQUIRES "esp_wifi_remote"
             "esp_timer"
             "freertos"
             "log"
             "sensor_fusion"
             "wifi_manager"
             "wifi_positioning"
)
