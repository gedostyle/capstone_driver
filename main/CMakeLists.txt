set(SRCS "main.cpp")

if(DEVICE STREQUAL "remote")
    list(APPEND SRCS "remote/remote.cpp")
elseif(DEVICE STREQUAL "v1")
    list(APPEND SRCS "V1_Motor/v1_motor.cpp")
elseif(DEVICE STREQUAL "v2")
    list(APPEND SRCS "V2_Motor/v2_motor.cpp")
endif()

idf_component_register(SRCS ${SRCS}
                    INCLUDE_DIRS "."
                    REQUIRES esp_adc driver esp_driver_gpio esp_driver_ledc esp_timer esp_driver_pcnt esp_driver_spi)