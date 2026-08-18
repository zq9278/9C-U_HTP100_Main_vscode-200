cmake_minimum_required(VERSION 3.22)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()

set(CMAKE_C_FLAGS_DEBUG "-O0 -g3")
set(CMAKE_C_FLAGS_RELEASE "-O2 -DNDEBUG")
set(CMAKE_PROJECT_NAME 9C-HTP100-main-v2)

include(cmake/gcc-arm-none-eabi.cmake)
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)
project(${CMAKE_PROJECT_NAME})
enable_language(C ASM)

add_executable(${CMAKE_PROJECT_NAME})
add_subdirectory(cmake/stm32cubemx)

file(GLOB APP_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/App/Src/*.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/Bsp/Src/*.c"
)

target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${APP_SOURCES})
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/App/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/Bsp/Inc
)
target_link_libraries(${CMAKE_PROJECT_NAME} stm32cubemx)
target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE
    -Wall -Wextra -Wshadow -Wdouble-promotion
    -Wno-unused-parameter
)
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE -Wl,--gc-sections)

add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.bin
)
