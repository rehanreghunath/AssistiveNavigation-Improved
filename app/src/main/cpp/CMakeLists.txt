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

find_package(OpenCV REQUIRED core imgproc video)

add_library(assistivenav SHARED
    FlowEngine.cpp
    jni_bridge.cpp
)

target_include_directories(assistivenav PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(assistivenav
    android
    log
    ${OpenCV_LIBS}
)