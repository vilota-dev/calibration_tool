set(GLFW_BUILD_EXAMPLES OFF CACHE INTERNAL "Build the GLFW example programs")
set(GLFW_BUILD_TESTS OFF CACHE INTERNAL "Build the GLFW test programs")
set(GLFW_BUILD_DOCS OFF CACHE INTERNAL "Build the GLFW documentation")
set(GLFW_INSTALL OFF CACHE INTERNAL "Generate installation target")

set(GUI_SOURCES
  "glad/src/glad.c"
  "imgui/imgui.cpp"
  "imgui/imgui_demo.cpp"
  "imgui/imgui_draw.cpp"
  "imgui/imgui_tables.cpp"
  "imgui/imgui_widgets.cpp"
  "imgui/backends/imgui_impl_glfw.cpp"
  "imgui/backends/imgui_impl_opengl3.cpp"
  "implot/implot.cpp"
  "implot/implot_items.cpp"
  "implot/implot_demo.cpp"
  "immvision/src_all_in_one/immvision/immvision.cpp"
  )

find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)
find_package(OpenCV REQUIRED)

add_library(gui STATIC ${GUI_SOURCES})

target_include_directories(gui PUBLIC
  "${CMAKE_CURRENT_SOURCE_DIR}/glad/include"
  "${CMAKE_CURRENT_SOURCE_DIR}/imgui"
  "${CMAKE_CURRENT_SOURCE_DIR}/imgui/backends/"
  "${CMAKE_CURRENT_SOURCE_DIR}/implot"
  "${CMAKE_CURRENT_SOURCE_DIR}/immvision/src_all_in_one/immvision"
  )

target_link_libraries(gui PUBLIC glfw OpenGL::GL ${OpenCV_LIBS})

#### Non-gui libraries
# Disable sophus tests and examples
set(BUILD_SOPHUS_TESTS OFF CACHE BOOL "Disable Sophus tests")
set(BUILD_SOPHUS_EXAMPLES OFF CACHE BOOL "Disable Sophus examples")

add_subdirectory("nativefiledialog_extended")
add_subdirectory("Sophus")
add_subdirectory("ros")

add_library(non_gui INTERFACE)

target_include_directories(non_gui INTERFACE
  "${CMAKE_CURRENT_SOURCE_DIR}/spdlog/include"
  "${CMAKE_CURRENT_SOURCE_DIR}/cereal/include"
  "${CMAKE_CURRENT_SOURCE_DIR}/Eigen"
  "${CMAKE_CURRENT_SOURCE_DIR}/nativefiledialog_extended/src/include"
  "${CMAKE_CURRENT_SOURCE_DIR}/basalt_headers/include"
  "${CMAKE_CURRENT_SOURCE_DIR}/Sophus"
  "${CMAKE_CURRENT_SOURCE_DIR}/ros"
  )

target_link_libraries(non_gui INTERFACE
  nfd
  Sophus::Sophus
  rosbag
  )
