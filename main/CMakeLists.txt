FILE(GLOB_RECURSE main ${CMAKE_SOURCE_DIR}/main/*.*)

idf_component_register(SRCS ${main}
                    PRIV_REQUIRES spi_flash
                    INCLUDE_DIRS ""
                    REQUIRES bsp_extra bsp_display bsp_illuminate bsp_i2c esp_timer esp_wifi nvs_flash lwip)
