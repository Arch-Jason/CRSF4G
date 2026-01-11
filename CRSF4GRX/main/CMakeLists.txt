idf_component_register(
    SRCS
        "main.cpp"
    PRIV_REQUIRES
        spi_flash
        driver
        esp_timer
    INCLUDE_DIRS ""
)
