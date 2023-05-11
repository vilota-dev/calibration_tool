cmake_minimum_required(VERSION 3.10)
project(calibration_tool)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Preprocessor Macros
add_definitions(-DPROJECT_ROOT="${CMAKE_SOURCE_DIR}")

# Find the required packages
find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)
find_package(OpenCV REQUIRED)

# External directories
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/external/glad/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/external/imgui)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/external/imgui/backends)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/external/implot)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/external/spdlog/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/external/basalt_headers/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/external/cereal/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/external/Eigen)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/external/immvision/src_all_in_one/immvision)

# External submodules (with their own CMakeLists.txt)
add_subdirectory("external/nativefiledialog_extended")
add_subdirectory("external/Sophus")

# Project submodules
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src/gui)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src/utils)

# Add source files
set(SOURCES
    src/main.cpp
    src/gui/gui.cpp
    src/utils/utils.cpp
    external/glad/src/glad.c
    external/imgui/imgui.cpp
    external/imgui/imgui_demo.cpp
    external/imgui/imgui_draw.cpp
    external/imgui/imgui_tables.cpp
    external/imgui/imgui_widgets.cpp
    external/imgui/backends/imgui_impl_glfw.cpp
    external/imgui/backends/imgui_impl_opengl3.cpp
    external/implot/implot.cpp
    external/implot/implot_items.cpp
    external/implot/implot_demo.cpp
    external/immvision/src_all_in_one/immvision/immvision.cpp
)

# Compile the executable
add_executable(${PROJECT_NAME} ${SOURCES})

# Link libraries
target_link_libraries(${PROJECT_NAME} PRIVATE 
  OpenGL::GL 
  glfw 
  nfd 
  Sophus::Sophus 
  ${OpenCV_LIBS}
  )
