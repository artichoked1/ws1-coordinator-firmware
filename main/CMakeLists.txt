set(SOURCES
    "main.cpp"
    "hal/sensorbus_esp_hal.c"
    "lorawan_storage.cpp"
    )

idf_component_register(SRCS ${SOURCES} INCLUDE_DIRS ".")
