idf_component_register(
    SRCS "slam_web_server.c"
    INCLUDE_DIRS "include"
    REQUIRES "esp_http_server"
             "esp_wifi"
             "json"
             "esp_timer"
             "freertos"
             "log"
             "communication"
)
