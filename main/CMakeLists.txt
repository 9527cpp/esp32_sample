# CONFIG_* is empty the first time this file is read (component requirements are
# collected before kconfig is imported). The choice is therefore also read from
# sdkconfig, falling back to sdkconfig.defaults, so a fresh tree still builds.
set(_sample "")
if(DEFINED PROJECT_DIR)
    set(_root "${PROJECT_DIR}")
else()
    set(_root "${CMAKE_SOURCE_DIR}")
endif()
foreach(_cfg "${_root}/sdkconfig.defaults" "${_root}/sdkconfig")
    if(EXISTS "${_cfg}")
        file(STRINGS "${_cfg}" _lines)
        foreach(_line IN LISTS _lines)
            if(_line STREQUAL "CONFIG_SAMPLE_HELLO=y")
                set(_sample "hello")
            elseif(_line STREQUAL "CONFIG_SAMPLE_BLINK=y")
                set(_sample "blink")
            elseif(_line STREQUAL "CONFIG_SAMPLE_RC522=y")
                set(_sample "rc522")
            elseif(_line STREQUAL "CONFIG_SAMPLE_MPU6050=y")
                set(_sample "mpu6050")
            elseif(_line STREQUAL "CONFIG_SAMPLE_BMP280=y")
                set(_sample "bmp280")
            endif()
        endforeach()
    endif()
endforeach()

if(CONFIG_SAMPLE_HELLO)
    set(_sample "hello")
elseif(CONFIG_SAMPLE_BLINK)
    set(_sample "blink")
elseif(CONFIG_SAMPLE_RC522)
    set(_sample "rc522")
elseif(CONFIG_SAMPLE_MPU6050)
    set(_sample "mpu6050")
elseif(CONFIG_SAMPLE_BMP280)
    set(_sample "bmp280")
endif()

if(_sample STREQUAL "")
    message(FATAL_ERROR "No sample selected. Set one under menuconfig -> Sample.")
endif()

set(_requires esp_driver_gpio)
if(_sample STREQUAL "rc522")
    list(APPEND _requires esp_driver_spi)
elseif(_sample STREQUAL "mpu6050" OR _sample STREQUAL "bmp280")
    list(APPEND _requires esp_driver_i2c)
endif()

idf_component_register(
    SRCS "app_main.c" "samples/${_sample}.c"
    INCLUDE_DIRS "."
    REQUIRES ${_requires}
)
