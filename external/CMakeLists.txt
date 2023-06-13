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

add_library(gui SHARED ${GUI_SOURCES})

target_include_directories(gui PUBLIC
        "${CMAKE_CURRENT_SOURCE_DIR}/glad/include"
        "${CMAKE_CURRENT_SOURCE_DIR}/imgui"
        "${CMAKE_CURRENT_SOURCE_DIR}/imgui/backends/"
        "${CMAKE_CURRENT_SOURCE_DIR}/implot"
        "${CMAKE_CURRENT_SOURCE_DIR}/immvision/src_all_in_one/immvision"
        )

target_link_libraries(gui PUBLIC glfw OpenGL::GL ${OpenCV_LIBS})

#### Non-gui libraries

add_subdirectory("nativefiledialog_extended")
add_subdirectory("ros")
add_subdirectory("apriltag")
add_subdirectory("libcbdetect")
add_subdirectory(spdlog)
find_package(TBB REQUIRED)
add_subdirectory("ecal-common/cpp")

add_library(non_gui INTERFACE)

target_include_directories(non_gui INTERFACE
        "${CMAKE_CURRENT_SOURCE_DIR}/spdlog/include"
        "${CMAKE_CURRENT_SOURCE_DIR}/nativefiledialog_extended/src/include"
        "${CMAKE_CURRENT_SOURCE_DIR}/basalt-headers/include"
        "${CMAKE_CURRENT_SOURCE_DIR}/basalt-headers/thirdparty/cereal/include"
        "${CMAKE_CURRENT_SOURCE_DIR}/basalt-headers/thirdparty/eigen"
        "${CMAKE_CURRENT_SOURCE_DIR}/basalt-headers/thirdparty/Sophus"
        "${CMAKE_CURRENT_SOURCE_DIR}/apriltag/include"
        "${CMAKE_CURRENT_SOURCE_DIR}/ros"
        "${OpenCV_INCLUDE_DIRS}"
        "${CMAKE_CURRENT_SOURCE_DIR}/libcbdetect/include"
        "${CMAKE_CURRENT_SOURCE_DIR}/ecal-common/cpp/include"
        )

if(NOT TBB_USE_GLIBCXX_VERSION AND UNIX AND NOT APPLE)
    if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
        string(REPLACE "." "0" TBB_USE_GLIBCXX_VERSION ${CMAKE_CXX_COMPILER_VERSION})
    endif()
    add_definitions(-DTBB_USE_GLIBCXX_VERSION=${TBB_USE_GLIBCXX_VERSION})
endif()

target_link_libraries(non_gui INTERFACE
        TBB::tbb
        spdlog::spdlog
        apriltag
        nfd
        rosbag
        cbdetect
        pthread
        ecal_camera::ecal_camera
        )

