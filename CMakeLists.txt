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

# Setup C++ compiler settings (ApplicationCpp 插件框架使用 C++17)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME Sentry_H7_Upper_CMake)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM, C and C++ languages
enable_language(C ASM CXX)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Collect user sources
file(GLOB_RECURSE USER_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/IMU/*.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/BSP/*.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/math/*.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/UserMiddlewares/*.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/Application/*.c"
)

# Collect user C++ sources (ApplicationCpp 插件框架)
file(GLOB_RECURSE USER_CPP_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/ApplicationCpp/*.cpp"
)

# Collect user C++ sources (Platform STM32 平台适配层)
file(GLOB_RECURSE USER_PLATFORM_CPP_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/Platform/*.cpp"
)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ${USER_SOURCES}
    ${USER_CPP_SOURCES}
    ${USER_PLATFORM_CPP_SOURCES}
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    #IMU
    ${CMAKE_CURRENT_SOURCE_DIR}/IMU/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/IMU/Algorithm/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/IMU/BMI088/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/IMU/Mahony/inc
    
    #BSP
    ${CMAKE_CURRENT_SOURCE_DIR}/BSP/inc

    #UserMiddlewares
    ${CMAKE_CURRENT_SOURCE_DIR}/UserMiddlewares/inc

    #Math
    ${CMAKE_CURRENT_SOURCE_DIR}/math/inc

    # Application
    ${CMAKE_CURRENT_SOURCE_DIR}/Application/inc

    # ApplicationCpp（C++17 插件框架）
    ${CMAKE_CURRENT_SOURCE_DIR}/ApplicationCpp
    ${CMAKE_CURRENT_SOURCE_DIR}/ApplicationCpp/Framework
    ${CMAKE_CURRENT_SOURCE_DIR}/ApplicationCpp/Platform
    ${CMAKE_CURRENT_SOURCE_DIR}/ApplicationCpp/Nodes

    # Config（编译期配置头，robot_build_config.h）
    ${CMAKE_CURRENT_SOURCE_DIR}/Config

    # Platform（C++ 平台适配层，STM32 具体实现）
    ${CMAKE_CURRENT_SOURCE_DIR}/Platform
    ${CMAKE_CURRENT_SOURCE_DIR}/Platform/STM32

    # CMSIS-DSP include paths   
    ${CMAKE_SOURCE_DIR}/Middlewares/ST/ARM/dsp/Include
    ${CMAKE_SOURCE_DIR}/Middlewares/ST/ARM/dsp/Inc
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    ${CMAKE_SOURCE_DIR}/Middlewares/ST/ARM/DSP/Lib/libCMSISDSP.a
    # Add user defined libraries
)
