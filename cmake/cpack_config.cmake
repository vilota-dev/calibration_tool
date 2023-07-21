string(TOLOWER "${PROJECT_NAME}" PROJECT_NAME_LOWERCASE)

find_program(DPKG_PROGRAM dpkg DOC "dpkg program of Debian-based systems")
if(DPKG_PROGRAM)
    execute_process(
        COMMAND ${DPKG_PROGRAM} --print-architecture
        OUTPUT_VARIABLE CPACK_DEBIAN_PACKAGE_ARCHITECTURE
        OUTPUT_STRIP_TRAILING_WHITESPACE)
endif(DPKG_PROGRAM)

find_program(LSB_RELEASE_PROGRAM lsb_release DOC "lsb_release program of Debian-based systems")
if(LSB_RELEASE_PROGRAM)
    execute_process(COMMAND ${LSB_RELEASE_PROGRAM} -rs
        OUTPUT_VARIABLE LSB_RELEASE_ID_SHORT
        OUTPUT_STRIP_TRAILING_WHITESPACE)

    if(${LSB_RELEASE_ID_SHORT} EQUAL "20.04")
        set(DEBIAN_DEPENDS "libtbb2, libopencv-core4.2, libopencv-imgproc4.2, libopencv-calib3d4.2, libusb-1.0-0-dev, ecal (>= ${eCAL_VERSION})")
    endif()

    if(${LSB_RELEASE_ID_SHORT} EQUAL "22.04")
        set(DEBIAN_DEPENDS "libtbb2, libopencv-core4.5d, libopencv-imgproc4.5d, libopencv-calib3d4.5d, libusb-1.0-0-dev, ecal (>= ${eCAL_VERSION})")
    endif()

endif(LSB_RELEASE_PROGRAM)

execute_process(
    COMMAND git log -1 --format=%h
    WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
    OUTPUT_VARIABLE GIT_HASH
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

string(TIMESTAMP PROJECT_VERSION_REVISION "%Y%m%d")

set(CPACK_VERBATIM_VARIABLES YES)

set(CPACK_GENERATOR "DEB;TGZ")
# set(CPACK_PROJECT_CONFIG_FILE ${CMAKE_SOURCE_DIR}/cmake/package.linux.txt) # Keep default location
set(CPACK_PACKAGE_VENDOR "Vilota Pte Ltd")
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "Huimin Cheng <huimin@vilota.ai>, Tejas Garrepally <tejas@vilota.ai>")
set(CPACK_PACKAGE_VERSION_MAJOR ${PROJECT_VERSION_MAJOR})
set(CPACK_PACKAGE_VERSION_MINOR ${PROJECT_VERSION_MINOR})
set(CPACK_PACKAGE_VERSION_PATCH "${PROJECT_VERSION_PATCH}-${PROJECT_VERSION_REVISION}-${GIT_HASH}~${LSB_RELEASE_ID_SHORT}")
set(CPACK_DEBIAN_PACKAGE_DEPENDS ${DEBIAN_DEPENDS})
set(CPACK_PACKAGE_FILE_NAME "${PROJECT_NAME_LOWERCASE}_${CPACK_PACKAGE_VERSION_MAJOR}.${CPACK_PACKAGE_VERSION_MINOR}.${CPACK_PACKAGE_VERSION_PATCH}_${CPACK_DEBIAN_PACKAGE_ARCHITECTURE}_${CMAKE_BUILD_TYPE}")
set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE")

set(CPACK_COMPONENTS_GROUPING ALL_COMPONENTS_IN_ONE)#ONE_PER_GROUP)
# without this you won't be able to pack only specified component
set(CPACK_DEB_COMPONENT_INSTALL YES)

include(CPack)