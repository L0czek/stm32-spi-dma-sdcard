cmake_minimum_required(VERSION 3.22)

project(stm32-spi-dma-sdcard C)

add_library(${PROJECT_NAME} STATIC
    src/sd_functions.c
)

target_include_directories(${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# Link to stm32cubemx to get HAL includes and defines
target_link_libraries(${PROJECT_NAME} PUBLIC stm32cubemx)
