cmake_minimum_required(VERSION 3.0)

set(CMAKE_INSTALL_PREFIX ${CMAKE_BINARY_DIR}/install CACHE PATH "Install location")
project(MAPTK)

option(MAPTK_SUPERBUILD  "Enable MAPTK superbuild" ON)

###
# CMake policies
#
if(POLICY CMP0054)
  cmake_policy(SET CMP0054 NEW)
endif()

###
# Options and setup
#
include(CMakeDependentOption)

###
# Add the CMake directory for CMake modules
#
list(INSERT CMAKE_MODULE_PATH 0 "${CMAKE_SOURCE_DIR}/CMake" )

###
# Versioning
#
set(MAPTK_VERSION_MAJOR 0)
set(MAPTK_VERSION_MINOR 11)
set(MAPTK_VERSION_PATCH 0)
set(MAPTK_VERSION "${MAPTK_VERSION_MAJOR}.${MAPTK_VERSION_MINOR}.${MAPTK_VERSION_PATCH}")
string(TIMESTAMP MAPTK_COPYRIGHT_YEAR "%Y")

###
# project specific directories
#
set(MAPTK_CMAKE_DIR            "${CMAKE_SOURCE_DIR}/CMake")

# root directories
set(MAPTK_SOURCE_DIR           "${CMAKE_SOURCE_DIR}")
set(MAPTK_BINARY_DIR           "${CMAKE_BINARY_DIR}")
set(MAPTK_STAMP_DIR ${MAPTK_BINARY_DIR}/MAPTK-ExternalProject-Stamps)

# Always build shared libraries, required due to plugin infrastructure
if(DEFINED BUILD_SHARED_LIBS AND NOT BUILD_SHARED_LIBS)
  message(FATAL_ERROR "MAP-Tk must be built as shared libraries, please set BUILD_SHARED_LIBS=ON")
endif()
set(BUILD_SHARED_LIBS ON)

set(LIB_SUFFIX "${MAPTK_LIB_SUFFIX}" CACHE STRING "String suffix appended to the library directory we install into")
mark_as_advanced( LIB_SUFFIX )

###
# Option for CUDA/CUDNN
#
option(MAPTK_BUILD_WITH_CUDA "Build with CUDA support" FALSE)
if (MAPTK_BUILD_WITH_CUDA)
  find_package( CUDA QUIET REQUIRED )
# Option for CUDNN
  option(MAPTK_BUILD_WITH_CUDNN "Build with CUDNN support" FALSE)
  if (MAPTK_BUILD_WITH_CUDNN)
    set( CUDNN_TOOLKIT_ROOT_DIR "" CACHE PATH "CUDNN root folder" )
    find_package( CUDNN QUIET REQUIRED)
  endif()
elseif(MAPTK_BUILD_WITH_CUDNN)
  unset(MAPTK_BUILD_WITH_CUDNN CACHE)
  message(WARNING "Disabling MAPTK_BUILD_WITH_CUDNN, You must have MAPTK_BUILD_WITH_CUDA enabled for this to be enabled")
endif()

if(MAPTK_SUPERBUILD)
  include(${MAPTK_CMAKE_DIR}/maptk-superbuild.cmake)
  return()
endif()

include( maptk-depends )

###
# use kwiver/fletch/maptk util methods
#
find_package(kwiver REQUIRED)
include_directories("${KWIVER_INCLUDE_DIRS}")
link_directories("${KWIVER_LIBRARY_DIRS}")
include( kwiver-cmake-future )
include( kwiver-utils )
include( kwiver-flags )
include( kwiver-configcheck )
find_package(fletch REQUIRED)
include( maptk-utils ) # local utilities

# set the name for our package exports
set(kwiver_export_name     maptk_exports)

add_subdirectory(maptk)

# this is where the executables source code lives
add_subdirectory(tools)

# this is where the algorithm default configuration files live
add_subdirectory(config)

# this is where the plugins to 3rd party software live
add_subdirectory(plugins)

# this is where configuration for example data sets lives
add_subdirectory(examples)

###
# GUI
#
option(MAPTK_ENABLE_GUI "Build MAPTK GUI" OFF)
if(MAPTK_ENABLE_GUI)
  if(APPLE)
    set(CMAKE_BUNDLE_LOCATION "${CMAKE_INSTALL_PREFIX}")
    # make sure CMAKE_INSTALL_PREFIX ends in /
    if(NOT CMAKE_INSTALL_PREFIX MATCHES "/$")
      set(CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}/")
    endif()
    set(CMAKE_INSTALL_PREFIX
      "${CMAKE_INSTALL_PREFIX}TeleSculptor.app/Contents")
  endif()
  add_subdirectory(gui)
endif()


###
# Manuals
#
option(MAPTK_ENABLE_MANUALS "Build MAPTK user manual(s)" OFF)
if(MAPTK_ENABLE_MANUALS)
  add_subdirectory(doc/manuals)
endif()


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
set(maptk_cmake_install_dir  "${CMAKE_INSTALL_PREFIX}/lib${LIB_SUFFIX}/cmake/maptk")

# Install rules for CMake utilities
include(maptk-install-utils)

# Prepare space-separated list of library names for config
get_property(maptk_libs GLOBAL PROPERTY kwiver_libraries)
string(REPLACE ";" " " maptk_libs "${maptk_libs}")

# Configure build-tree CMake config file and export associated targets file
set(MAPTK_CONFIG_FILE "${MAPTK_BINARY_DIR}/maptk-config.cmake")
set(module_path "${MAPTK_SOURCE_DIR}/CMake")
kwiver_configure_file( maptk-config
  "${MAPTK_SOURCE_DIR}/CMake/maptk-config.cmake.in"
  "${MAPTK_CONFIG_FILE}"
  MAPTK_SOURCE_DIR
  MAPTK_BINARY_DIR
  EIGEN3_INCLUDE_DIR
  MAPTK_VERSION
  maptk_libs
  module_path
  )
kwiver_export_targets("${MAPTK_BINARY_DIR}/maptk-config-targets.cmake")

# Configure install-tree CMake config file and export associated targets file
set(MAPTK_CONFIG_INSTALL_FILE "${MAPTK_BINARY_DIR}/maptk-config-install.cmake")
set(module_path "${CMAKE_INSTALL_PREFIX}/lib${LIB_SUFFIX}/cmake/maptk")
kwiver_configure_file( maptk-install-config
  "${MAPTK_SOURCE_DIR}/CMake/maptk-config-install.cmake.in"
  "${MAPTK_CONFIG_INSTALL_FILE}"
  EIGEN3_INCLUDE_DIR
  MAPTK_VERSION
  maptk_libs
  module_path
  )
kwiver_install(
  FILES       "${MAPTK_CONFIG_INSTALL_FILE}"
  DESTINATION "${maptk_cmake_install_dir}"
  RENAME      maptk-config.cmake
  )
kwiver_install(
  EXPORT      ${kwiver_export_name}
  DESTINATION "${maptk_cmake_install_dir}"
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



set(CPACK_PACKAGE_NAME              "MAP-Tk")
set(CPACK_PACKAGE_VENDOR            "Kitware, Inc.")
set(CPACK_PACKAGE_CONTACT           "kitware@kitware.com")
set(CPACK_MONOLITHIC_INSTALL        true)
set(CPACK_PACKAGE_VERSION_MAJOR     "${MAPTK_VERSION_MAJOR}")
set(CPACK_PACKAGE_VERSION_MINOR     "${MAPTK_VERSION_MINOR}")
set(CPACK_PACKAGE_VERSION_PATCH     "${MAPTK_VERSION_PATCH}")
set(CPACK_PACKAGE_VERSION           "${MAPTK_VERSION}")
set(CPACK_RESOURCE_FILE_LICENSE     "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE")
set(CPACK_PACKAGE_FILE_NAME         "${CPACK_PACKAGE_NAME}-${CPACK_PACKAGE_VERSION}-${CPACK_SYSTEM_NAME}")
set(CPACK_PACKAGE_ICON              "${CMAKE_CURRENT_SOURCE_DIR}/packaging\\\\maptk_install.bmp")
set(CPACK_DMG_DS_STORE              "${CMAKE_CURRENT_SOURCE_DIR}/packaging/DS_Store")

# NSIS Package options
set(CPACK_NSIS_INSTALLED_ICON_NAME "bin\\\\TeleSculptor.exe")
set(CPACK_NSIS_MUI_ICON "${CMAKE_CURRENT_SOURCE_DIR}/gui/icons\\\\telesculptor.ico")
set(CPACK_NSIS_MUI_UNIICON "${CMAKE_CURRENT_SOURCE_DIR}/gui/icons\\\\telesculptor.ico")
set(CPACK_NSIS_CONTACT "${CPACK_PACKAGE_CONTACT}")
set(CPACK_NSIS_HELP_LINK "http://www.kitware.com")
set(CPACK_NSIS_URL_INFO_ABOUT "http://www.kwiver.org")
set(CPACK_NSIS_MENU_LINKS "http://www.kwiver.org" "KWIVER Website")

if(MAPTK_ENABLE_GUI)
  set (CPACK_PACKAGE_EXECUTABLES "TeleSculptor" "MAP-Tk TeleSculptor" ${CPACK_PACKAGE_EXECUTABLES})
endif()

if(APPLE)
  set(CPACK_BINARY_DRAGNDROP ON)
  set(CPACK_BINARY_PACKAGEMAKER OFF)
  set(CPACK_SET_DESTDIR TRUE)
  set(CPACK_PACKAGE_ICON "${CMAKE_CURRENT_SOURCE_DIR}/packaging/maptk_drive.icns")
endif()
include (CPack)
