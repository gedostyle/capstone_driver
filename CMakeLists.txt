# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.22)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

idf_build_set_property(COMPILE_OPTIONS "-Wno-error" APPEND)
project(capstone_motor_driver)
add_compile_options(-Wno-error=overloaded-virtual)
if(DEFINED DEVICE)
    if(DEVICE STREQUAL "v1")
        add_compile_definitions(DEVICE_v1_MOTOR_CONTROLLER)
    elseif(DEVICE STREQUAL "v2")
        add_compile_definitions(DEVICE_v2_MOTOR_CONTROLLER)
    elseif(DEVICE STREQUAL "remote")
        add_compile_definitions(DEVICE_REMOTE_MOTOR_CONTROLLER)
    else()
        message(FATAL_ERROR "Unknown DEVICE value '${DEVICE}'. Use v1, v2, or remote.")
    endif()
else()
    add_compile_definitions(DEVICE_v1_MOTOR_CONTROLLER)
endif()