cmake_minimum_required(VERSION 2.8.3)
project(MAPTK)

###
# Versioning
#
set(MAPTK_VERSION_MAJOR 0)
set(MAPTK_VERSION_MINOR 1)
set(MAPTK_VERSION_PATCH 2)
set(MAPTK_VERSION "${MAPTK_VERSION_MAJOR}.${MAPTK_VERSION_MINOR}.${MAPTK_VERSION_PATCH}")


###
# Add the CMake directory for CMake modules
#
list(INSERT CMAKE_MODULE_PATH 0
  "${MAPTK_SOURCE_DIR}/cmake"
  )


###
# Options and setup
#
include(CMakeDependentOption)

# we can't build shared libraries on Windows so we leave it off by default;
# we haven't set up DLL exports or anything like that
if (WIN32)
  option(BUILD_SHARED_LIBS    "Build shared libraries."                       ON)
else()
  option(BUILD_SHARED_LIBS    "Build shared libraries."                       OFF)
endif()

option(MAPTK_ENABLE_OPENCV  "Enable OpenCV-dependent code"                    OFF)
option(MAPTK_ENABLE_VXL     "Enable VXL-dependent code"                       OFF)
option(MAPTK_ENABLE_VISCL   "Enable VisCL dependent code (for OpenCL on GPU)" OFF)
option(MAPTK_ENABLE_PROJ    "Enable PROJ dependent code"                      OFF)



###
# MAPTK CMake config
#
include(maptk-utils)
include(maptk-flags)

set(maptk_export_name maptk_exports)


###
# Dependencies and Modules
#

if(WIN32)
  set(Boost_USE_STATIC_LIBS TRUE)
endif()
find_package(Boost 1.46.1 REQUIRED COMPONENTS system filesystem program_options)
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

if(MAPTK_ENABLE_VISCL)
  find_package(viscl REQUIRED)
  include_directories(SYSTEM ${viscl_INCLUDE_DIR})
endif()

if(MAPTK_ENABLE_PROJ)
  find_package(PROJ)
endif()

# this is where the libraries source code lives
add_subdirectory(maptk)

# this is where the executables source code lives
add_subdirectory(tools)

###
# Testing
#

# The following are required to uses Dart and the Cdash dashboard
option(MAPTK_ENABLE_TESTING "Build MAPTK testing" OFF)
if(MAPTK_ENABLE_TESTING)
  enable_testing()
  include(CTest)
  set(BUILD_TESTING ON)
  mark_as_advanced(BUILD_TESTING)
  add_subdirectory(tests)
endif()


###
# Top level installation
#

# Configure build-tree CMake config file and export associated targets file
set(MAPTK_CONFIG_FILE "${MAPTK_BINARY_DIR}/maptk-config.cmake")
maptk_configure_file(maptk-config
  "${MAPTK_SOURCE_DIR}/cmake/maptk-config.cmake.in"
  "${MAPTK_CONFIG_FILE}"
  MAPTK_SOURCE_DIR
  )
maptk_export_targets("${MAPTK_BINARY_DIR}/maptk-config-targets.cmake")

# Configure install-tree CMake config file and export associated targets file
set(cmake_install_dir lib/cmake/maptk)
set(MAPTK_CONFIG_INSTALL_FILE "${MAPTK_BINARY_DIR}/maptk-config-install.cmake")
maptk_configure_file(maptk-install-config
  "${MAPTK_SOURCE_DIR}/cmake/maptk-config-install.cmake.in"
  "${MAPTK_CONFIG_INSTALL_FILE}"
  )
maptk_install(
  FILES       "${MAPTK_CONFIG_INSTALL_FILE}"
  DESTINATION "${cmake_install_dir}"
  RENAME      maptk-config.cmake
  )
maptk_install(
  EXPORT      ${maptk_export_name}
  DESTINATION "${cmake_install_dir}"
  FILE        maptk-config-targets.cmake
  )


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

set(CPACK_PACKAGE_NAME              "MAPTK")
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
