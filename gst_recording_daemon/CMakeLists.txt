cmake_minimum_required(VERSION 3.16)
project(gst_recording_daemon LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(PkgConfig REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED IMPORTED_TARGET gstreamer-1.0 gstreamer-base-1.0)

add_executable(gst_recording_daemon
  src/main.cpp
  src/RecorderDaemon.cpp
)

target_include_directories(gst_recording_daemon PRIVATE src)
target_link_libraries(gst_recording_daemon PRIVATE PkgConfig::GSTREAMER)

target_compile_options(gst_recording_daemon PRIVATE
  -Wall
  -Wextra
  -Wpedantic
)
