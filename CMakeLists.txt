cmake_minimum_required(VERSION 3.10)
project(calibration_tool)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_REQUIRED_FLAGS "-Wno-error")

include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/build_helpers.cmake")
detectOS()

if (APPLE)
   # Do nothing
elseif (UNIX AND NOT APPLE)
    set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
    set(CMAKE_INSTALL_RPATH "$ORIGIN")
    if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
        set(CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}/install" CACHE PATH "Installation Directory" FORCE)
    endif()
endif ()

# Preprocessor Macros
add_definitions(-DPROJECT_ROOT="${CMAKE_SOURCE_DIR}")
if (APPLE)
    set(OPENSSL_INCLUDE_DIR "/opt/homebrew/opt/openssl@3/include")
    set(OPENSSL_CRYPTO_LIBRARY "/opt/homebrew/opt/openssl@3/lib")
    set(OPENSSL_ROOT_DIR "/opt/homebrew/opt/openssl@3")
endif ()
add_compile_definitions(SOPHUS_USE_BASIC_LOGGING)

# Project submodules
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/external/ecal-common/cpp/include)
add_subdirectory("external")

# Add source files
set(SOURCES src/main.cpp src/calibration/calibrator.cpp src/recorder/dataset.cpp)

# Compile the executable
add_executable(${PROJECT_NAME} ${APPLICATION_TYPE} ${SOURCES})

# Link libraries
target_link_libraries(${PROJECT_NAME} PRIVATE gui non_gui)


# Installs
if(UNIX AND NOT APPLE)
    install(TARGETS ${PROJECT_NAME}
            RUNTIME DESTINATION bin
            )

    install(TARGETS gui non_gui cbdetect
            LIBRARY DESTINATION lib
            )

    set(CPACK_PACKAGE_NAME "${PROJECT_NAME}")
    set(CPACK_PACKAGE_VERSION "1.0.0")
    set(CPACK_DEBIAN_PACKAGE_MAINTAINER "Tejas") # replace with your name
    set(CPACK_GENERATOR "DEB")
    include(CPack)
endif()