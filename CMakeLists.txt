# Create project
cmake_minimum_required(VERSION 3.16.0)
project(WolfVision VERSION 2021.7.9 LANGUAGES CXX)

# Specialize CMake build flags for different build types
set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

# Use standard c++20
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Prefer posix threads
set(THREADS_PREFER_PTHREAD_FLAG True)
find_package(Threads REQUIRED)

# Find and add opencv as required
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# Add fmtlib directory and include path for target
add_subdirectory(3rdparty/fmt EXCLUDE_FROM_ALL)

# Add fmtlib include to EXTRA_INCLUDES
list(APPEND EXTRA_INCLUDES 3rdparty/fmt/include)

# Add mindvision linux sdk
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    # Add mindvision-sdk directory and include path for target
    add_library(MVSDK SHARED IMPORTED)
    get_filename_component(MVSDK_PATH 3rdparty/mindvision/linux REALPATH)

    # Check system architecture for mindvision-sdk library
    EXECUTE_PROCESS(COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE)
    message(STATUS "Checking platform architecture for MVSDK: ${ARCHITECTURE}")
    if(${ARCHITECTURE} STREQUAL x86_64)
        set(MIMDVISION_SDK_LIBRARY ${MVSDK_PATH}/lib/x64/libMVSDK.so)
    elseif(${ARCHITECTURE} STREQUAL x86)
        set(MIMDVISION_SDK_LIBRARY ${MVSDK_PATH}/lib/x86/libMVSDK.so)
    elseif(${ARCHITECTURE} STREQUAL aarch64)
        set(MIMDVISION_SDK_LIBRARY ${MVSDK_PATH}/lib/arm64/libMVSDK.so)
    elseif(${ARCHITECTURE} STREQUAL armv7)
        set(MIMDVISION_SDK_LIBRARY ${MVSDK_PATH}/lib/arm/libMVSDK.so)
    else()
        message(FATAL_ERROR "Not supported architecture for MVSDK: ${ARCHITECTURE}")
    endif()
    set_property(TARGET MVSDK PROPERTY IMPORTED_LOCATION ${MIMDVISION_SDK_LIBRARY})

    # Add mindvision lib to build target
    list(APPEND EXTRA_LIBS MVSDK)

    # Add mindvision include path
    list(APPEND EXTRA_INCLUDES 3rdparty/mindvision/linux/include)
endif()

# Add base src and header
list(APPEND EXTRA_INCLUDES base)

# Add device as shared library to build target
list(APPEND EXTRA_INCLUDES devices)

list(APPEND EXTRA_INCLUDES ${PROJECT_SOURCE_DIR}/devices/camera)
add_library(wv-mv-video-capture SHARED devices/camera/mv_video_capture.cpp)
list(APPEND EXTRA_LIBS wv-mv-video-capture)

list(APPEND EXTRA_INCLUDES ${PROJECT_SOURCE_DIR}/devices/serial)
add_library(wv-uart-serial SHARED devices/serial/uart_serial.cpp)
list(APPEND EXTRA_LIBS wv-uart-serial)
add_library(wv-can-bus SHARED devices/serial/can_bus.cpp)
list(APPEND EXTRA_LIBS wv-can-bus)

# Add module as shared library to build target
list(APPEND EXTRA_INCLUDES module)

list(APPEND EXTRA_INCLUDES ${PROJECT_SOURCE_DIR}/module/angle_solve)
add_library(wv-basic-pnp SHARED module/angle_solve/basic_pnp.cpp)
list(APPEND EXTRA_LIBS wv-basic-pnp)

list(APPEND EXTRA_INCLUDES ${PROJECT_SOURCE_DIR}/module/armor)
add_library(wv-basic-armor SHARED module/armor/basic_armor.cpp)
list(APPEND EXTRA_LIBS wv-basic-armor)
add_library(wv-fan-armor SHARED module/armor/fan_armor.cpp)
list(APPEND EXTRA_LIBS wv-fan-armor)

list(APPEND EXTRA_INCLUDES ${PROJECT_SOURCE_DIR}/module/blade)
add_library(wv-basic-blade SHARED module/blade/basic_blade.cpp)
list(APPEND EXTRA_LIBS wv-basic-blade)

list(APPEND EXTRA_INCLUDES ${PROJECT_SOURCE_DIR}/module/buff)
add_library(wv-basic-buff SHARED module/buff/basic_buff.cpp)
list(APPEND EXTRA_LIBS wv-basic-buff)

list(APPEND EXTRA_INCLUDES ${PROJECT_SOURCE_DIR}/module/filter)
add_library(wv-basic-kalman SHARED module/filter/basic_kalman.cpp)
list(APPEND EXTRA_LIBS wv-basic-kalman)

list(APPEND EXTRA_INCLUDES ${PROJECT_SOURCE_DIR}/module/object_detection)
add_library(wv-yolov4-dnn SHARED module/object_detection/yolov4_dnn.cpp)
list(APPEND EXTRA_LIBS wv-yolov4-dnn)

list(APPEND EXTRA_INCLUDES ${PROJECT_SOURCE_DIR}/module/roi)
add_library(wv-basic-roi SHARED module/roi/basic_roi.cpp)
list(APPEND EXTRA_LIBS wv-basic-roi)

# Add utils header
list(APPEND EXTRA_INCLUDES utils)

# Add EXTRA_INCLUDES to project includes
include_directories(${EXTRA_INCLUDES})

# Add base/wolfvision to executable
add_executable(WolfVision base/wolfvision.cpp)

# Add shared library link to target
target_link_libraries(WolfVision
    PUBLIC
    Threads::Threads
    fmt::fmt-header-only
    ${OpenCV_LIBS}

    PRIVATE
    ${EXTRA_LIBS}
)

# Set WolfVision compile features to standared c+20
target_compile_features(WolfVision PUBLIC cxx_std_20)

# Set WolfVision linking time optimization to true
set_target_properties(WolfVision PROPERTIES INTERPROCEDURAL_OPTIMIZATION True)

# Set built binary to ~/bin
set(EXECUTABLE_OUTPUT_PATH "${PROJECT_BINARY_DIR}/bin")

# Set built library to ~/lib
set(LIBRARY_OUTPUT_PATH "${PROJECT_BINARY_DIR}/lib")

# Add test_camera directory
add_subdirectory(test/test_camera)
