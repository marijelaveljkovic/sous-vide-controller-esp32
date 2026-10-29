idf_component_register(SRCS "mvhttp.c" "mvwifi.c" "mvlcd.c" "mvtemperature.c" "mvbeeper.c" "mvencoder.c" "main.c"
                       PRIV_REQUIRES spi_flash esp_driver_gpio esp_timer driver esp_wifi esp_event esp_netif nvs_flash esp_http_server
                       INCLUDE_DIRS "")
