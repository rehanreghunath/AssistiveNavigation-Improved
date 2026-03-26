cmake_minimum_required(VERSION 3.22.1)
project("assistivenav" LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_options(
    -O3
    -ffast-math
    -march=armv8-a
    -fvisibility=hidden
)

# ── OpenCV ─────────────────────────────────────────────────────────────────────
find_package(OpenCV REQUIRED core imgproc video)

# ── OpenAL-Soft ────────────────────────────────────────────────────────────────

if(NOT DEFINED OPENAL_DIR)
    message(FATAL_ERROR
        "OPENAL_DIR is not set.\n"
        "Add 'openal.dir=/path/to/openal-soft' to local.properties.\n"
        "See CMakeLists.txt for build instructions.")
endif()

# Prebuilt OpenAL shared library
add_library(openal SHARED IMPORTED)
set_target_properties(openal PROPERTIES
    IMPORTED_LOCATION
        "${OPENAL_DIR}/build-android/${ANDROID_ABI}/libopenal.so"
)

# ── Native library ─────────────────────────────────────────────────────────────

# 1. DEFINE TARGET FIRST
add_library(assistivenav SHARED
    FlowEngine.cpp
    GridAnalyzer.cpp
    ImuFusion.cpp
    AudioEngine.cpp
    jni_bridge.cpp
)

# 2. INCLUDE DIRECTORIES (AFTER TARGET EXISTS)
target_include_directories(assistivenav PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${OPENAL_DIR}/include
)

# 3. LINK LIBRARIES
target_link_libraries(assistivenav
    android
    log
    ${OpenCV_LIBS}
    openal
)