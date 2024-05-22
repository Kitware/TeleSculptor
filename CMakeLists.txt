cmake_minimum_required(VERSION 3.9)

project(TeleSculptor-SuperBuild)

option(TELESCULPTOR_SUPERBUILD  "Enable TeleSculptor superbuild" ON)

###
# CMake policies
#
if(POLICY CMP0020)
  cmake_policy(SET CMP0020 NEW)
endif()
if(POLICY CMP0054)
  cmake_policy(SET CMP0054 NEW)
endif()
if(POLICY CMP0071)
  cmake_policy(SET CMP0071 NEW)
endif()
# This policy is new in CMake 3.12. The NEW behavior uses the <PackageName>_ROOT variable in
# find_package(<PackageName>) calls.
# See: https://cmake.org/cmake/help/git-stage/policy/CMP0074.html
if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

###
# Options and setup
#
include(CMakeDependentOption)

###
# Add the CMake directory for CMake modules
#
list(INSERT CMAKE_MODULE_PATH 0 "${CMAKE_SOURCE_DIR}/CMake" )

string(TIMESTAMP TELESCULPTOR_COPYRIGHT_YEAR "%Y")

###
# project specific directories
#
set(TELESCULPTOR_CMAKE_DIR      "${CMAKE_SOURCE_DIR}/CMake")

# root directories
set(TELESCULPTOR_SOURCE_DIR     "${CMAKE_SOURCE_DIR}")
set(TELESCULPTOR_BINARY_DIR     "${CMAKE_BINARY_DIR}")
set(TELESCULPTOR_EXTERNAL_DIR   "${TELESCULPTOR_BINARY_DIR}/external")
set(TELESCULPTOR_STAMP_DIR       ${TELESCULPTOR_EXTERNAL_DIR}/stamps)

set(CMAKE_CXX_STANDARD 17)

if(NOT CMAKE_BUILD_TYPE)
  message(STATUS "Setting build type to 'Release' as none was specified.")
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build." FORCE)
endif()
SET_PROPERTY(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS Debug Release RelWithDebInfo)

get_property(CMAKE_GENERATOR_IS_MULTI_CONFIG GLOBAL
    PROPERTY GENERATOR_IS_MULTI_CONFIG)
if(CMAKE_GENERATOR_IS_MULTI_CONFIG)
  message(STATUS "Generator is multi-config, setting configuration types")
  if(${CMAKE_BUILD_TYPE} STREQUAL "Debug")
    set(CMAKE_CONFIGURATION_TYPES "Debug" CACHE STRING INTERNAL FORCE )
  else() # Provide both configurations as they are compatible with each other
    set(CMAKE_CONFIGURATION_TYPES "Release" "RelWithDebInfo" CACHE STRING INTERNAL FORCE )
  endif()
endif()

# Always build shared libraries, required due to plugin infrastructure
if(DEFINED BUILD_SHARED_LIBS AND NOT BUILD_SHARED_LIBS)
  message(FATAL_ERROR "TeleSculptor must be built as shared libraries, please set BUILD_SHARED_LIBS=ON")
endif()
set(BUILD_SHARED_LIBS ON)

set(LIB_SUFFIX "${TELESCULPTOR_LIB_SUFFIX}"
    CACHE STRING "String suffix appended to the library directory we install into")
mark_as_advanced( LIB_SUFFIX )

set(LIBRARY_PATH_VAR "LD_LIBRARY_PATH")
if( APPLE )
  set(LIBRARY_PATH_VAR "DYLD_FALLBACK_LIBRARY_PATH")
endif()

option(TELESCULPTOR_ENABLE_MANUALS "Build TeleSculptor user manual(s)" OFF)
option(TELESCULPTOR_ENABLE_TESTING "Build TeleSculptor testing" OFF)
set(PYTHON_VERSION_MAJOR "3")

if(TELESCULPTOR_SUPERBUILD)
  include(${TELESCULPTOR_CMAKE_DIR}/telesculptor-superbuild.cmake)
  return()
endif()
project(TeleSculptor VERSION 1.2.0)
include( telesculptor-depends )

# set the name for our package exports
set(kwiver_export_name     telesculptor_exports)

add_subdirectory(maptk)

# this is where the algorithm default configuration files live
add_subdirectory(config)

# this is where the plugins to 3rd party software live
add_subdirectory(plugins)

# this is where configuration for example data sets lives
add_subdirectory(examples)

###
# GUI
#
if(APPLE)
  set(CMAKE_BUNDLE_LOCATION "/")
  set(CMAKE_INSTALL_PREFIX "/TeleSculptor.app/Contents")
endif()
add_subdirectory(gui)

###
# Environment Scripts
#
configure_file(${TELESCULPTOR_CMAKE_DIR}/start_MSVC.bat.in
               ${TELESCULPTOR_BINARY_DIR}/start_MSVC.bat @ONLY)
configure_file(${TELESCULPTOR_CMAKE_DIR}/setup_TeleSculptor.build.bat.in
               ${TELESCULPTOR_BINARY_DIR}/setup_TeleSculptor.bat @ONLY)
configure_file(${TELESCULPTOR_CMAKE_DIR}/setup_TeleSculptor.build.sh.in
               ${TELESCULPTOR_BINARY_DIR}/setup_TeleSculptor.sh @ONLY)

if(MSVC)
  # Create vcxproj.user files for every executable with the correct runtime environment
  kwiver_setup_msvc_env(${TELESCULPTOR_BINARY_DIR}/setup_TeleSculptor.bat)
endif()

###
# Manuals
#
if(TELESCULPTOR_ENABLE_MANUALS)
  add_subdirectory(doc)
endif()

###
# Testing
#
# The following are required to uses Dart and the Cdash dashboard
if(TELESCULPTOR_ENABLE_TESTING)
  enable_testing()
  include(CTest)
  set(BUILD_TESTING ON)
  mark_as_advanced(BUILD_TESTING)
  add_subdirectory(tests)
endif()

###
# Top level installation
#
set(telesculptor_cmake_install_dir  "lib${LIB_SUFFIX}/cmake/telesculptor")

# Install rules for CMake utilities
include(telesculptor-install-utils)

# Prepare space-separated list of library names for config
get_property(telesculptor_libs GLOBAL PROPERTY kwiver_libraries)
string(REPLACE ";" " " telesculptor_libs "${telesculptor_libs}")

# Configure build-tree CMake config file and export associated targets file
set(TELESCULPTOR_CONFIG_FILE "${TELESCULPTOR_BINARY_DIR}/telesculptor-config.cmake")
set(module_path "${TELESCULPTOR_SOURCE_DIR}/CMake")
kwiver_configure_file( telesculptor-config
  "${TELESCULPTOR_SOURCE_DIR}/CMake/telesculptor-config.cmake.in"
  "${TELESCULPTOR_CONFIG_FILE}"
  TELESCULPTOR_SOURCE_DIR
  TELESCULPTOR_BINARY_DIR
  TeleSculptor_VERSION
  telesculptor_libs
  module_path
  )
kwiver_export_targets("${TELESCULPTOR_BINARY_DIR}/telesculptor-config-targets.cmake")

# Configure install-tree CMake config file and export associated targets file
set(TELESCULPTOR_CONFIG_INSTALL_FILE
    "${TELESCULPTOR_BINARY_DIR}/telesculptor-config-install.cmake")
set(module_path "${CMAKE_INSTALL_PREFIX}/lib${LIB_SUFFIX}/cmake/telesculptor")
kwiver_configure_file( telesculptor-install-config
  "${TELESCULPTOR_SOURCE_DIR}/CMake/telesculptor-config-install.cmake.in"
  "${TELESCULPTOR_CONFIG_INSTALL_FILE}"
  TeleSculptor_VERSION
  telesculptor_libs
  module_path
  )
kwiver_install(
  FILES       "${TELESCULPTOR_CONFIG_INSTALL_FILE}"
  DESTINATION "${telesculptor_cmake_install_dir}"
  RENAME      telesculptor-config.cmake
  )
kwiver_install(
  EXPORT      ${kwiver_export_name}
  DESTINATION "${telesculptor_cmake_install_dir}"
  FILE        telesculptor-config-targets.cmake
  )

###
# CPack Packaging
#

#TODO: Define package dependencies
set(TELESCULPTOR_DEPS "")

if(EXISTS /etc/redhat-release)
  file(READ /etc/redhat-release RHEL_VERSION)
  string(REGEX REPLACE ".*release ([^\\. ]*).*" "\\1" RHEL_VERSION "${RHEL_VERSION}")
  set(CPACK_SYSTEM_NAME "el${RHEL_VERSION}.${CMAKE_SYSTEM_PROCESSOR}")
  set(CPACK_RPM_PACKAGE_AUTOREQPROV " no")
  set(CPACK_RPM_PACKAGE_REQUIRES "${TELESCULPTOR_DEPS}")
else()
  set(CPACK_SYSTEM_NAME "${CMAKE_SYSTEM_NAME}-${CMAKE_SYSTEM_PROCESSOR}")
endif()



set(CPACK_PACKAGE_NAME              "TeleSculptor")
set(CPACK_PACKAGE_VENDOR            "Kitware, Inc.")
set(CPACK_PACKAGE_CONTACT           "kitware@kitware.com")
set(CPACK_MONOLITHIC_INSTALL        true)
set(CPACK_PACKAGE_VERSION_MAJOR     "${TeleSculptor_VERSION_MAJOR}")
set(CPACK_PACKAGE_VERSION_MINOR     "${TeleSculptor_VERSION_MINOR}")
set(CPACK_PACKAGE_VERSION_PATCH     "${TeleSculptor_VERSION_PATCH}")
set(CPACK_PACKAGE_VERSION           "${TeleSculptor_VERSION}")
set(CPACK_RESOURCE_FILE_LICENSE     "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE")
set(CPACK_PACKAGE_FILE_NAME         "${CPACK_PACKAGE_NAME}-${CPACK_PACKAGE_VERSION}-${CPACK_SYSTEM_NAME}")
set(CPACK_PACKAGE_ICON              "${CMAKE_CURRENT_SOURCE_DIR}/packaging\\\\telesculptor_install.bmp")
set(CPACK_DMG_DS_STORE              "${CMAKE_CURRENT_SOURCE_DIR}/packaging/DS_Store")

# NSIS Package options
set(CPACK_NSIS_INSTALLED_ICON_NAME "bin\\\\TeleSculptor.exe")
set(CPACK_NSIS_MUI_ICON "${CMAKE_CURRENT_SOURCE_DIR}/gui/icons\\\\telesculptor.ico")
set(CPACK_NSIS_MUI_UNIICON "${CMAKE_CURRENT_SOURCE_DIR}/gui/icons\\\\telesculptor.ico")
set(CPACK_NSIS_CONTACT "${CPACK_PACKAGE_CONTACT}")
set(CPACK_NSIS_HELP_LINK "http://www.kitware.com")
set(CPACK_NSIS_URL_INFO_ABOUT "http://www.kwiver.org")
set(CPACK_NSIS_MENU_LINKS "http://www.kwiver.org" "KWIVER Website")

set(CPACK_PACKAGE_EXECUTABLES "TeleSculptor" "TeleSculptor" ${CPACK_PACKAGE_EXECUTABLES})

if(APPLE)
  set(CPACK_GENERATOR "DragNDrop")
  set(CPACK_BINARY_PACKAGEMAKER OFF)
  set(CPACK_SET_DESTDIR TRUE)
  set(CPACK_PACKAGE_ICON "${CMAKE_CURRENT_SOURCE_DIR}/packaging/telesculptor_drive.icns")
endif()
include (CPack)
