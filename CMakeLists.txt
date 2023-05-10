cmake_minimum_required(VERSION 3.10)
project(calibration_tool)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find the required packages
find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)

# External directories
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/external/imgui)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/external/imgui/backends)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/external/implot)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/external/spdlog/include)

# External submodules (with their own CMakeLists.txt)
add_subdirectory("external/nativefiledialog_extended")

# Project submodules
# include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src/terminal)

# Add source files
set(SOURCES
    src/main.cpp
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
)

# Compile the executable
add_executable(${PROJECT_NAME} ${SOURCES})

# Link libraries
target_link_libraries(${PROJECT_NAME} PRIVATE OpenGL::GL glfw nfd)


