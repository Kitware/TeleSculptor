cmake_minimum_required( VERSION 2.8 )
project( MAPTK )

###
# Versioning
#
set(MAPTK_VERSION_MAJOR 0)
set(MAPTK_VERSION_MINOR 1)
set(MAPTK_VERSION_PATCH 0)
set(MAPTK_VERSION "${MAPTK_VERSION_MAJOR}.${MAPTK_VERSION_MINOR}.${MAPTK_VERSION_PATCH}")


###
# Add the CMake directory for CMake modules
#
set(CMAKE_MODULE_PATH "${MAPTK_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})


###
# Cross-platform path determination
#
set(MAPTK_BUILD_DEST_BIN        ${CMAKE_BINARY_DIR}/bin)
set(MAPTK_BUILD_DEST_DOC        ${CMAKE_BINARY_DIR}/doc)
set(MAPTK_BUILD_DEST_LIB        ${CMAKE_BINARY_DIR}/lib)

set(MAPTK_INSTALL_DEST_BIN      ${CMAKE_INSTALL_PREFIX}/bin)
set(MAPTK_INSTALL_DEST_INCLUDE  ${CMAKE_INSTALL_PREFIX}/include)
set(MAPTK_INSTALL_DEST_DOC      ${CMAKE_INSTALL_PREFIX}/share/doc/${CMAKE_PROJECT_NAME}-${MAPTK_VERSION})
set(MAPTK_INSTALL_DEST_LIB      ${CMAKE_INSTALL_PREFIX}/lib${LIB_SUFFIX})

set(MAPTK_INSTALL_TARGETS_ARGS_COMMON
  EXPORT maptk
  RUNTIME DESTINATION ${MAPTK_INSTALL_DEST_BIN}
  LIBRARY DESTINATION ${MAPTK_INSTALL_DEST_LIB}
  ARCHIVE DESTINATION ${MAPTK_INSTALL_DEST_LIB}
)

if(NOT CMAKE_ARCHIVE_OUTPUT_DIRECTORY)
  set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${MAPTK_BUILD_DEST_LIB})
endif()
if(NOT CMAKE_LIBRARY_OUTPUT_DIRECTORY)
  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${MAPTK_BUILD_DEST_LIB})
endif()
if(NOT CMAKE_RUNTIME_OUTPUT_DIRECTORY)
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${MAPTK_BUILD_DEST_BIN})
endif()


###
# Options
#

# we can't build shared libraries on Windows so we leave it off by default;
# we haven't set up DLL exports or anything like that
option(BUILD_SHARED_LIBS    "Build shared libraries."                         OFF)

option(MAPTK_ENABLE_OPENCV  "Enable OpenCV-dependent code"                    OFF)
option(MAPTK_ENABLE_VXL     "Enable VXL-dependent code"                       OFF)
option(MAPTK_ENABLE_VISCL   "Enable VisCL dependent code (for OpenCL on GPU)" OFF)


###
# MAPTK CMake config and export file
#

# Set up the build export configuration
set(MAPTK_EXPORT_FILE "${MAPTK_BINARY_DIR}/maptk-config.cmake")
configure_file(
  "${MAPTK_SOURCE_DIR}/cmake/maptk-config.cmake.in"
  "${MAPTK_EXPORT_FILE}"
)

# Set up the install export
set(MAPTK_EXPORT_INSTALL_FILE "${MAPTK_BINARY_DIR}/CMakeFiles/maptk-config.cmake")
set(MAPTK_EXPORT_INSTALL_TARGETS_FILE "maptk-config-targets.cmake")
configure_file(
  "${MAPTK_SOURCE_DIR}/cmake/maptk-config-install.cmake.in"
  "${MAPTK_EXPORT_INSTALL_FILE}"
  @ONLY
)


###
# Dependencies and Modules
#
include(maptk-doxygen)

if(WIN32)
  set(Boost_USE_STATIC_LIBS True)
endif()
find_package(Boost 1.42 REQUIRED COMPONENTS thread filesystem system date_time regex)
add_definitions(-DBOOST_ALL_NO_LIB)
include_directories(SYSTEM ${Boost_INCLUDE_DIRS})

if(MAPTK_ENABLE_OPENCV)
  find_package(OpenCV REQUIRED)
  include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS})
  # Docs say we don't need to add link_directories() call for OpenCV lib dirs
endif()
set(USE_OPENCV_VERSION MAPTK_ENABLE_OPENCV AND OpenCV_FOUND AND OpenCV_VERSION VERSION_GREATER "2.4")

if(MAPTK_ENABLE_VXL)
  find_package(VXL REQUIRED)
  include_directories(SYSTEM ${VXL_VCL_INCLUDE_DIR})
  include_directories(SYSTEM ${VXL_CORE_INCLUDE_DIR})
  link_directories(${VXL_LIBRARY_DIR})
endif()

if( MAPTK_ENABLE_VISCL )
  find_package(viscl REQUIRED)
  include_directories(SYSTEM ${viscl_INCLUDE_DIR})
endif( MAPTK_ENABLE_VISCL )

# this is where the main source code lives
add_subdirectory(maptk)


###
# Testing
#

# The following are required to uses Dart and the Cdash dashboard
enable_testing()
include(CTest)
if(BUILD_TESTING)
  add_subdirectory(tests)
endif(BUILD_TESTING)


###
# Top level installation
#

# install CMake configuration files
install(FILES ${MAPTK_EXPORT_INSTALL_FILE}
  DESTINATION lib/cmake
)
install(EXPORT maptk
  DESTINATION lib/cmake
  FILE maptk-config-targets.cmake
)

# TODO: If the build doc directory exists, install it.


###
# CPack Packaging
#

#TODO: Define package dependencies
set(MAPTK_DEPS "")

if(EXISTS /etc/redhat-release)
  file(READ /etc/redhat-release RHEL_VERSION)
  string(REGEX REPLACE ".*release ([^\\. ]*).*" "\\1" RHEL_VERSION "${RHEL_VERSION}")
  set(CPACK_SYSTEM_NAME "el${RHEL_VERSION}.${CMAKE_SYSTEM_PROCESSOR}")
  set(CPACK_RPM_PACKAGE_AUTOREQPROV " no")
  set(CPACK_RPM_PACKAGE_REQUIRES "${MAPTK_DEPS}")

else()
  set(CPACK_SYSTEM_NAME "${CMAKE_SYSTEM_NAME}-${CMAKE_SYSTEM_PROCESSOR}")
endif()

set(CPACK_PACKAGE_NAME              "maptk")
set(CPACK_PACKAGE_VENDOR            "Kitware, Inc.")
set(CPACK_PACKAGE_CONTACT           "kitware@kitware.com")
set(CPACK_MONOLITHIC_INSTALL        true)
set(CPACK_PACKAGE_VERSION_MAJOR     "${MAPTK_VERSION_MAJOR}")
set(CPACK_PACKAGE_VERSION_MINOR     "${MAPTK_VERSION_MINOR}")
set(CPACK_PACKAGE_VERSION_PATCH     "${MAPTK_VERSION_PATCH}")
set(CPACK_PACKAGE_VERSION           "${MAPTK_VERSION}")
set(CPACK_RESOURCE_FILE_LICENSE     "${CMAKE_CURRENT_SOURCE_DIR}/KITWARE_LICENSE.txt")
set(CPACK_PACKAGING_INSTALL_PREFIX  "${CMAKE_INSTALL_PREFIX}")
set(CPACK_PACKAGE_FILE_NAME         "${CPACK_PACKAGE_NAME}-${CPACK_PACKAGE_VERSION}-${CPACK_SYSTEM_NAME}")
include (CPack)
