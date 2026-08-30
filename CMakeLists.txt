cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME Xiaomi_CyberDog1)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# CubeMX regenerates cmake/stm32cubemx/CMakeLists.txt and always adds the
# STM32F1 HAL CAN driver. This target-level replacement lives in this
# non-generated top-level file, so every configure uses the GD32 CAN patch.
set(STM32_HAL_CAN_SOURCE
    "${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_can.c"
)
set(GD32_HAL_CAN_SOURCE
    "${CMAKE_CURRENT_SOURCE_DIR}/Code_Source/BSP/stm32f1xx_hal_can_gd32.c"
)

if(NOT EXISTS "${GD32_HAL_CAN_SOURCE}")
    message(FATAL_ERROR "GD32 CAN patch file not found: ${GD32_HAL_CAN_SOURCE}")
endif()

get_target_property(STM32_DRIVER_SOURCES STM32_Drivers SOURCES)
list(FILTER STM32_DRIVER_SOURCES EXCLUDE REGEX "[/\\\\]stm32f1xx_hal_can\\.c$")
set_property(TARGET STM32_Drivers PROPERTY SOURCES "${STM32_DRIVER_SOURCES}")
target_sources(STM32_Drivers PRIVATE "${GD32_HAL_CAN_SOURCE}")

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
# Collect all source files from Code_Source, excluding duplicate I2C implementations
file(GLOB_RECURSE CODE_SOURCE_SRC_ALL
    ${CMAKE_CURRENT_SOURCE_DIR}/Code_Source/**/*.c
)
# Exclude SoftIIC_Fast and SoftIIC_General implementations (use MID_SoftIIC wrapper instead)
list(FILTER CODE_SOURCE_SRC_ALL EXCLUDE REGEX "SoftIIC_Fast/")
list(FILTER CODE_SOURCE_SRC_ALL EXCLUDE REGEX "SoftIIC_General/")
# Exclude drv835x.c — conflicts with drv83xx.c (both define global stru_config)
list(FILTER CODE_SOURCE_SRC_ALL EXCLUDE REGEX "BSP/DRV8353/")
# Exclude stm32f1xx_hal_can_gd32.c — already compiled in STM32_Drivers via CubeMX CMakeLists
list(FILTER CODE_SOURCE_SRC_ALL EXCLUDE REGEX "stm32f1xx_hal_can_gd32\\.c")
set(CODE_SOURCE_SRC ${CODE_SOURCE_SRC_ALL})

file(GLOB_RECURSE CMSIS_DSP_SRC
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS/DSP/Source/**/*.c
)
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    ${CODE_SOURCE_SRC}
    ${CMSIS_DSP_SRC}
    # Simulink 生成的 FOC 算法代码（单精度）
    ${CMAKE_CURRENT_SOURCE_DIR}/Simulink_Model/generated/CyberDog_Motor_FOC.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Simulink_Model/generated/CyberDog_Motor_FOC_data.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Simulink_Model/generated/rt_nonfinite.c
    # FreeRTOS Kernel（固定版本 V11.3.0，见 FreeRTOS-Kernel/README.md）
    ${CMAKE_CURRENT_SOURCE_DIR}/FreeRTOS-Kernel/tasks.c
    ${CMAKE_CURRENT_SOURCE_DIR}/FreeRTOS-Kernel/queue.c
    ${CMAKE_CURRENT_SOURCE_DIR}/FreeRTOS-Kernel/list.c
    ${CMAKE_CURRENT_SOURCE_DIR}/FreeRTOS-Kernel/timers.c
    ${CMAKE_CURRENT_SOURCE_DIR}/FreeRTOS-Kernel/event_groups.c
    ${CMAKE_CURRENT_SOURCE_DIR}/FreeRTOS-Kernel/stream_buffer.c
    ${CMAKE_CURRENT_SOURCE_DIR}/FreeRTOS-Kernel/portable/GCC/ARM_CM4F/port.c
    # 注意：configSUPPORT_DYNAMIC_ALLOCATION=0，不链接任何 MemMang heap_*.c（内核不引用 pvPortMalloc）
)

# Add include paths
# IMPORTANT: Core/Inc must come before Code_Source paths to prevent
# header shadowing (e.g. Code_Source/BSP/CAN.h vs Core/Inc/can.h)
target_include_directories(${CMAKE_PROJECT_NAME} BEFORE PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS/DSP/Include
    ${CMAKE_CURRENT_SOURCE_DIR}/Code_Source/APP
    ${CMAKE_CURRENT_SOURCE_DIR}/Code_Source/BSP
    ${CMAKE_CURRENT_SOURCE_DIR}/Code_Source/BSP/DRV8353
    ${CMAKE_CURRENT_SOURCE_DIR}/Code_Source/COM
    ${CMAKE_CURRENT_SOURCE_DIR}/Code_Source/SERVICE
    ${CMAKE_CURRENT_SOURCE_DIR}/Code_Source/CONFIG
    ${CMAKE_CURRENT_SOURCE_DIR}/Code_Source/MID/MID_foc
    ${CMAKE_CURRENT_SOURCE_DIR}/Code_Source/MID/MID_SoftIIC
    ${CMAKE_CURRENT_SOURCE_DIR}/Code_Source/MID/MID_SoftIIC/SoftIIC_Fast
    ${CMAKE_CURRENT_SOURCE_DIR}/Code_Source/MID/MID_SoftIIC/SoftIIC_General
    ${CMAKE_CURRENT_SOURCE_DIR}/Code_Source/MID/MID_cm_backtrace
    ${CMAKE_CURRENT_SOURCE_DIR}/Code_Source/MID/MID_segger_rtt
    ${CMAKE_CURRENT_SOURCE_DIR}/Code_Source/MID/MID_segger_sysview
    ${CMAKE_CURRENT_SOURCE_DIR}/Code_Source/MID/MID_identify
    # Simulink 生成的 FOC 算法头文件
    ${CMAKE_CURRENT_SOURCE_DIR}/Simulink_Model/generated
    # FreeRTOS Kernel 头文件与端口
    ${CMAKE_CURRENT_SOURCE_DIR}/FreeRTOS-Kernel/include
    ${CMAKE_CURRENT_SOURCE_DIR}/FreeRTOS-Kernel/portable/GCC/ARM_CM4F
)

# Add project symbols (macros)
# GD32_PATCH: ARM_MATH_CM4 for Cortex-M4F with FPU (was ARM_MATH_CM3)
# __FPU_PRESENT enables the CMSIS DSP FPU codepath
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
    ARM_MATH_CM4
    __FPU_PRESENT=1U
)

# GD32_PATCH: add FPU compile and link options (override cortex-m3 from toolchain)
target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE
    -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16
    -save-temps
)
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
    -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16
    -u _printf_float
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
