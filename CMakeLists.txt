# SPDX-License-Identifier: Apache-2.0

# specifiy a minimum cmake version
cmake_minimum_required(VERSION 3.20.0)

list(APPEND ZEPHYR_EXTRA_MODULES ${CMAKE_CURRENT_SOURCE_DIR}/modules/libmicroros)

# locate the Zephyr RTOS source
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

# name and discribe project
project(
    app
    VERSION 1.0
    DESCRIPTION "sensor PJ project that will drive a stepper motor"
    LANGUAGES C
)


# locate the source code for the application
target_sources(app PRIVATE src/main.c)
