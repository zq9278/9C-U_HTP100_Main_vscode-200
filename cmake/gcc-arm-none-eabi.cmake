set(CMAKE_SYSTEM_NAME               Generic)
set(CMAKE_SYSTEM_PROCESSOR          arm)

set(CMAKE_C_COMPILER_ID GNU)
set(CMAKE_CXX_COMPILER_ID GNU)

# Some default GCC settings
# arm-none-eabi- must be part of path environment
set(TOOLCHAIN_PREFIX                arm-none-eabi-)

set(STM32CUBE_BUNDLE_ROOTS "")
foreach(STM32CUBE_BUNDLE_ROOT IN ITEMS
    "$ENV{STM32CUBE_BUNDLE_PATH}"
    "$ENV{CUBE_BUNDLE_PATH}"
    "$ENV{HOME}/Library/Application Support/stm32cube/bundles"
    "$ENV{LOCALAPPDATA}/stm32cube/bundles"
    "$ENV{HOME}/.stm32cube/bundles"
)
    if(EXISTS "${STM32CUBE_BUNDLE_ROOT}")
        list(APPEND STM32CUBE_BUNDLE_ROOTS "${STM32CUBE_BUNDLE_ROOT}")
    endif()
endforeach()

set(STM32CUBE_TOOLCHAIN_BINS "")
foreach(STM32CUBE_BUNDLE_ROOT IN LISTS STM32CUBE_BUNDLE_ROOTS)
    file(GLOB STM32CUBE_TOOLCHAIN_BINS_FOR_ROOT
        "${STM32CUBE_BUNDLE_ROOT}/gnu-tools-for-stm32/*/bin"
    )
    list(APPEND STM32CUBE_TOOLCHAIN_BINS ${STM32CUBE_TOOLCHAIN_BINS_FOR_ROOT})
endforeach()
list(SORT STM32CUBE_TOOLCHAIN_BINS COMPARE NATURAL ORDER DESCENDING)

find_program(ARM_NONE_EABI_GCC ${TOOLCHAIN_PREFIX}gcc HINTS ${STM32CUBE_TOOLCHAIN_BINS} NO_DEFAULT_PATH)
find_program(ARM_NONE_EABI_GXX ${TOOLCHAIN_PREFIX}g++ HINTS ${STM32CUBE_TOOLCHAIN_BINS} NO_DEFAULT_PATH)
find_program(ARM_NONE_EABI_OBJCOPY ${TOOLCHAIN_PREFIX}objcopy HINTS ${STM32CUBE_TOOLCHAIN_BINS} NO_DEFAULT_PATH)
find_program(ARM_NONE_EABI_SIZE ${TOOLCHAIN_PREFIX}size HINTS ${STM32CUBE_TOOLCHAIN_BINS} NO_DEFAULT_PATH)

if(NOT ARM_NONE_EABI_GCC OR NOT ARM_NONE_EABI_GXX OR NOT ARM_NONE_EABI_OBJCOPY OR NOT ARM_NONE_EABI_SIZE)
    message(FATAL_ERROR
        "arm-none-eabi toolchain not found. Set PATH or STM32CUBE_BUNDLE_PATH/CUBE_BUNDLE_PATH "
        "to your STM32Cube bundles directory."
    )
endif()

if(NOT ARM_NONE_EABI_GCC OR NOT ARM_NONE_EABI_GXX OR NOT ARM_NONE_EABI_OBJCOPY OR NOT ARM_NONE_EABI_SIZE)
    message(FATAL_ERROR
        "arm-none-eabi toolchain not found. Set PATH or STM32CUBE_BUNDLE_PATH/CUBE_BUNDLE_PATH "
        "to your STM32Cube bundles directory."
    )
endif()

set(CMAKE_C_COMPILER                ${ARM_NONE_EABI_GCC})
set(CMAKE_ASM_COMPILER              ${CMAKE_C_COMPILER})
set(CMAKE_CXX_COMPILER              ${ARM_NONE_EABI_GXX})
set(CMAKE_LINKER                    ${ARM_NONE_EABI_GXX})
set(CMAKE_OBJCOPY                   ${ARM_NONE_EABI_OBJCOPY})
set(CMAKE_SIZE                      ${ARM_NONE_EABI_SIZE})

set(CMAKE_EXECUTABLE_SUFFIX_ASM     ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_C       ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_CXX     ".elf")

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# MCU specific flags
set(TARGET_FLAGS "-mcpu=cortex-m0plus ")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${TARGET_FLAGS}")
set(CMAKE_ASM_FLAGS "${CMAKE_C_FLAGS} -x assembler-with-cpp -MMD -MP")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -Wpedantic -fdata-sections -ffunction-sections")

set(CMAKE_C_FLAGS_DEBUG "-O0 -g3")
set(CMAKE_C_FLAGS_RELEASE "-Os -g0")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g3")
set(CMAKE_CXX_FLAGS_RELEASE "-Os -g0")

set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -fno-rtti -fno-exceptions -fno-threadsafe-statics")

set(CMAKE_C_LINK_FLAGS "${TARGET_FLAGS}")
set(CMAKE_C_LINK_FLAGS "${CMAKE_C_LINK_FLAGS} -T \"${CMAKE_SOURCE_DIR}/STM32G070XX_FLASH.ld\"")
set(CMAKE_C_LINK_FLAGS "${CMAKE_C_LINK_FLAGS} --specs=nano.specs")
set(CMAKE_C_LINK_FLAGS "${CMAKE_C_LINK_FLAGS} -Wl,-Map=${CMAKE_PROJECT_NAME}.map -Wl,--gc-sections")
set(CMAKE_C_LINK_FLAGS "${CMAKE_C_LINK_FLAGS} -Wl,--start-group -lc -lm -Wl,--end-group")
set(CMAKE_C_LINK_FLAGS "${CMAKE_C_LINK_FLAGS} -Wl,--print-memory-usage")

set(CMAKE_CXX_LINK_FLAGS "${CMAKE_C_LINK_FLAGS} -Wl,--start-group -lstdc++ -lsupc++ -Wl,--end-group")
