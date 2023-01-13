cmake_minimum_required(VERSION 3.15)
project(esp32imu VERSION 0.1)

set(CMAKE_CXX_STANDARD 14)
if(NOT CMAKE_BUILD_TYPE)
    # Options: Debug, Release, MinSizeRel, RelWithDebInfo
    message(STATUS "No build type selected, default to Release")
    set(CMAKE_BUILD_TYPE "Release")
endif()

include(FetchContent)

###############################################################################
# Options
###############################################################################

option(BUILD_SHARED_LIBS "Build shared library" ON)
option(BUILD_BINDINGS_PYTHON "Build Python bindings" ON)

###############################################################################
# async_comm
###############################################################################

FetchContent_Declare(async_comm
  GIT_REPOSITORY  https://github.com/plusk01/async_comm
  GIT_TAG         msvc-fixes
)
FetchContent_MakeAvailable(async_comm)

###############################################################################
# argparse
###############################################################################

FetchContent_Declare(argparse
  GIT_REPOSITORY  https://github.com/p-ranav/argparse
  GIT_TAG         v2.9
)
FetchContent_MakeAvailable(argparse)

###############################################################################
# Pybind11
###############################################################################

if(BUILD_BINDINGS_PYTHON)
  FetchContent_Declare(pybind11
    GIT_REPOSITORY  https://github.com/pybind/pybind11
    GIT_TAG         v2.10.3
  )
  FetchContent_MakeAvailable(pybind11)

  add_subdirectory(bindings/python)
endif()

###############################################################################
# Targets
###############################################################################

add_library(esp32imu src/abstract_driver.cpp src/serial_driver.cpp src/udp_driver.cpp)
set_target_properties(esp32imu PROPERTIES
  POSITION_INDEPENDENT_CODE ON
  VERSION ${PROJECT_VERSION})
target_link_libraries(esp32imu PRIVATE async_comm)
target_include_directories(esp32imu PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>)
target_compile_definitions(esp32imu PUBLIC PROJECT_VERSION="${PROJECT_VERSION}")


add_executable(screenprint src/screenprint.cpp)
target_link_libraries(screenprint esp32imu argparse)
target_compile_definitions(screenprint PUBLIC PROJECT_VERSION="${PROJECT_VERSION}")
