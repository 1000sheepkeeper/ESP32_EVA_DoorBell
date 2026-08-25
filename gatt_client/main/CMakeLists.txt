idf_component_register(SRCS "gattc_demo.c" "doorbell_media.c"
                    PRIV_REQUIRES bt nvs_flash esp_driver_gpio esp_driver_spi
                                  esp_driver_sdspi esp_driver_i2s esp_timer
                                  fatfs sdmmc esp_lcd lvgl
                    INCLUDE_DIRS "."
                    EMBED_FILES "background.rgb565")
