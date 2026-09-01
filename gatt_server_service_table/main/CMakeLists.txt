idf_component_register(SRCS "gatts_table_creat_demo.c" "audio_player.c"
                    PRIV_REQUIRES bt nvs_flash esp_driver_gpio esp_driver_i2s esp_timer
                    INCLUDE_DIRS ".")
