cmake_minimum_required(VERSION 3.10)
project(calibration_tool VERSION 1.1.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_REQUIRED_FLAGS "-Wno-error")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")  # cmake default: "-O3 -DNDEBUG"
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/" ${CMAKE_MODULE_PATH})
option(HWSG "Enable only corner detector widget" OFF)
option(TRACY_ENABLE "Enable profiling" ON)

execute_process(
    COMMAND git rev-parse --abbrev-ref HEAD
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    OUTPUT_VARIABLE CURRENT_BRANCH
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

if(CURRENT_BRANCH STREQUAL "hwsg_working")
    message(STATUS "Building only corner detector widget")
    add_definitions(-DHWSG)
endif()

# configure build types and build c-flags
if(NOT CMAKE_BUILD_TYPE)
    # Release, RelWithAssert, RelWithEigenNan
    set(CMAKE_BUILD_TYPE RelWithDebInfo) # Choice of Release (ignore assert) and RelWithDebInfo (includes assert)
endif()
message(STATUS "CMAKE_BUILD_TYPE = ${CMAKE_BUILD_TYPE}")

# Set default installation directory
if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    set(CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}/install" CACHE PATH "Installation Directory" FORCE)
endif()

include(build_helpers)

detectOS()

# Preprocessor Macros
add_definitions(-DPROJECT_ROOT="${CMAKE_SOURCE_DIR}")
if (APPLE)
    set(OPENSSL_INCLUDE_DIR "/opt/homebrew/opt/openssl@3/include")
    set(OPENSSL_CRYPTO_LIBRARY "/opt/homebrew/opt/openssl@3/lib")
    set(OPENSSL_ROOT_DIR "/opt/homebrew/opt/openssl@3")
endif ()
add_compile_definitions(SOPHUS_USE_BASIC_LOGGING)

# Project submodules
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/external/ecal-common/cpp/include)
add_subdirectory("external")

# Add source files
set(SOURCES
        src/main.cpp
        src/app_state.cpp
        src/calibration/calibrator.cpp
        src/recorder/dataset.cpp
        src/recorder/presets.cpp
        src/utils/utils.cpp
        src/ui/window.cpp
        src/ui/view.cpp
        src/ui/views/view_corner_detector.cpp
        src/ui/views/view_recorder.cpp
        src/ui/views/view_rosbag_inspector.cpp)

# Compile the executable
add_executable(${PROJECT_NAME} ${APPLICATION_TYPE} ${SOURCES})

# Link libraries
target_link_libraries(${PROJECT_NAME} PRIVATE gui non_gui)

# TODO: Temporary, change once vk_calibrate receives prior path directly
set(KB4_PRIOR ${CMAKE_SOURCE_DIR}/priors/calibration-prior-kb4.json)
set(RADTAN_PRIOR ${CMAKE_SOURCE_DIR}/priors/calibration-prior-radtan8.json)

# Installs
install(TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME})
set_target_properties(${PROJECT_NAME} PROPERTIES INSTALL_RPATH "$ORIGIN/../lib" )

install(TARGETS gui non_gui
        LIBRARY DESTINATION lib)

set(PRIOR_FILES
    ${CMAKE_SOURCE_DIR}/priors/vk180-prior.json
    ${CMAKE_SOURCE_DIR}/priors/vk180-prior-ds.json
    ${CMAKE_SOURCE_DIR}/priors/vkl-m12-prior.json
    ${CMAKE_SOURCE_DIR}/priors/vkl-m12-prior-ds.json
    ${CMAKE_SOURCE_DIR}/priors/calibration-prior-kb4.json
    ${CMAKE_SOURCE_DIR}/priors/calibration-prior-radtan8.json)

install(FILES ${PRIOR_FILES} DESTINATION bin/priors)
# TODO: Remove this once vk_calibrate receives path directly.
#install(FILES ${KB4_PRIOR} DESTINATION /bin/priors/kb4)
#install(FILES ${RADTAN_PRIOR} DESTINATION /bin/priors/radtan)


include(cpack_config)