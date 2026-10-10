cmake_minimum_required(VERSION 3.22)

# This file is generated only once by CubeMX and may be extended by the user.

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

set(CMAKE_PROJECT_NAME Sentry_H7_Upper_CMake)
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

enable_language(C CXX ASM)

add_executable(${CMAKE_PROJECT_NAME})

add_subdirectory(cmake/stm32cubemx)

file(GLOB_RECURSE USER_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/IMU/*.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/BSP/*.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/math/*.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/UserMiddlewares/*.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/Application/*.c"
)

file(GLOB_RECURSE USER_CPP_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/ApplicationCpp/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/Platform/*.cpp"
)

# Only the diagnostic comparison requires IEEE finite-value checks.
set_source_files_properties(
    "${CMAKE_CURRENT_SOURCE_DIR}/ApplicationCpp/Nodes/Diagnostics/ComparisonDemoNode.cpp"
    PROPERTIES COMPILE_OPTIONS "-fno-fast-math"
)
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE)

target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ${USER_SOURCES}
    ${USER_CPP_SOURCES}
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/Config
    ${CMAKE_CURRENT_SOURCE_DIR}/ApplicationCpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Platform

    ${CMAKE_CURRENT_SOURCE_DIR}/IMU/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/IMU/Algorithm/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/IMU/BMI088/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/IMU/Mahony/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/BSP/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/UserMiddlewares/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/math/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/Application/inc
    ${CMAKE_SOURCE_DIR}/Middlewares/ST/ARM/dsp/Include
    ${CMAKE_SOURCE_DIR}/Middlewares/ST/ARM/dsp/Inc
)

target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE)

target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE
    $<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions;-fno-rtti>
)

list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    ${CMAKE_SOURCE_DIR}/Middlewares/ST/ARM/DSP/Lib/libCMSISDSP.a
)
