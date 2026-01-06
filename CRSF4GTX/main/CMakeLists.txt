idf_component_register(
    SRCS
        "wifi.cpp"
        "wifi.hpp"
        "main.cpp"
        "main.hpp"
        "CRSF.cpp"
        "CRSF.hpp"
        "udp.cpp"
        "udp.hpp"

    PRIV_REQUIRES
        esp_wifi
        esp_netif
        nvs_flash
        driver
        lwip
    INCLUDE_DIRS "."
)
