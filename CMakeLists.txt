idf_component_register(
  SRC_DIRS "src"
  INCLUDE_DIRS "include"
  REQUIRES driver esp_driver_gpio esp_driver_spi esp_driver_uart esp_driver_i2c loggable pn532_cxx
)

target_compile_definitions(${COMPONENT_LIB} PUBLIC "-DLOG_LOCAL_LEVEL=ESP_LOG_VERBOSE")
