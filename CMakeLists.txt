cmake_minimum_required(VERSION 3.10)
project(calibration_tool)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_REQUIRED_FLAGS "-Wno-error")

# Preprocessor Macros
add_definitions(-DPROJECT_ROOT="${CMAKE_SOURCE_DIR}")

# Project submodules
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)
add_subdirectory("external")

# Add source files
set(SOURCES
  src/main.cpp
  src/io/dataset_io.cpp
  src/calibration/aprilgrid.cpp
  )

# Compile the executable
add_executable(${PROJECT_NAME} ${SOURCES})

# Link libraries
target_link_libraries(${PROJECT_NAME} PRIVATE
  gui
  non_gui
  )
