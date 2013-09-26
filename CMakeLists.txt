project( maptk_all )

cmake_minimum_required( VERSION 2.8 )

set(maptk_source_dir
  "${CMAKE_CURRENT_SOURCE_DIR}")
set(maptk_binary_dir
  "${CMAKE_CURRENT_BINARY_DIR}")

set(maptk_VERSION_MAJOR 0)
set(maptk_VERSION_MINOR 1)
set(maptk_VERSION_PATCH 0)
set(maptk_VERSION "${maptk_VERSION_MAJOR}.${maptk_VERSION_MINOR}.${maptk_VERSION_PATCH}")

# add the CMake directory for CMake modules
set(CMAKE_MODULE_PATH "${maptk_all_SOURCE_DIR}/CMake" ${CMAKE_MODULE_PATH})

set(EXECUTABLE_OUTPUT_PATH    ${CMAKE_BINARY_DIR}/bin)
if (WIN32)
  set(LIBRARY_OUTPUT_PATH       ${CMAKE_BINARY_DIR}/bin)
else (WIN32)
  set(LIBRARY_OUTPUT_PATH       ${CMAKE_BINARY_DIR}/lib)
endif (WIN32)
set(PYTHON_OUTPUT_PATH        ${LIBRARY_OUTPUT_PATH}/python)

if (WIN32)
  set(EXECUTABLE_INSTALL_PATH    ${CMAKE_INSTALL_PREFIX}/${PREFIX}/bin)
  set(LIBRARY_INSTALL_PATH       ${CMAKE_INSTALL_PREFIX}/${PREFIX}/bin)
else (WIN32)
  set(EXECUTABLE_INSTALL_PATH    ${CMAKE_INSTALL_PREFIX}/${PREFIX}/bin)
  set(LIBRARY_INSTALL_PATH       ${CMAKE_INSTALL_PREFIX}/${PREFIX}/lib${LIB_SUFFIX})
endif (WIN32)
set(INCLUDE_INSTALL_PATH       ${CMAKE_INSTALL_PREFIX}/${PREFIX}/include)
set(DOCUMENTATION_INSTALL_PATH ${CMAKE_INSTALL_PREFIX}/${PREFIX}/share/doc/${CMAKE_PROJECT_NAME}-${maptk_version})


# we can't build shared libraries on Windows so we leave it off by default;
# we haven't set up DLL exports or anything like that
option(BUILD_SHARED_LIBS "Build with shared libraries." OFF)


if( NOT CMAKE_ARCHIVE_OUTPUT_DIRECTORY )
  set( CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${maptk_all_BINARY_DIR}/lib )
endif()
if( NOT CMAKE_LIBRARY_OUTPUT_DIRECTORY )
  set( CMAKE_LIBRARY_OUTPUT_DIRECTORY ${maptk_all_BINARY_DIR}/lib )
endif()
if( NOT CMAKE_RUNTIME_OUTPUT_DIRECTORY )
  set( CMAKE_RUNTIME_OUTPUT_DIRECTORY ${maptk_all_BINARY_DIR}/bin )
endif()

# Set up the build export configuration
set( MAPTK_EXPORT_FILE "${maptk_all_BINARY_DIR}/maptk-config.cmake" )
configure_file(
  "${maptk_all_SOURCE_DIR}/conf/maptk-config.cmake.in"
  "${MAPTK_EXPORT_FILE}"
)

# Set up the install export
set( MAPTK_EXPORT_INSTALL_FILE "${maptk_all_BINARY_DIR}/CMakeFiles/maptk-config.cmake" )
configure_file(
  "${maptk_all_SOURCE_DIR}/conf/maptk-config-install.cmake.in"
  "${MAPTK_EXPORT_INSTALL_FILE}"
  @ONLY
)


# The following are required to uses Dart and the Cdash dashboard
enable_testing()
include(CTest)

option( MAPTK_ENABLE_OPENCV "Enable OpenCV-dependent code" "OFF" )
option( MAPTK_ENABLE_VXL "Enable VXL-dependent code" "OFF" )
option( MAPTK_ENABLE_VISCL "Enable VisCL dependent code (for OpenCL on GPU)" "OFF" )

if( MAPTK_ENABLE_OPENCV )
  find_package( OpenCV REQUIRED )
  include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS})
endif( MAPTK_ENABLE_OPENCV )
set( USE_OPENCV_VERSION MAPTK_ENABLE_OPENCV AND OpenCV_FOUND AND OpenCV_VERSION VERSION_GREATER "2.4" )

if( MAPTK_ENABLE_VXL )
  find_package(VXL REQUIRED)
  include_directories(SYSTEM ${VXL_VCL_INCLUDE_DIR})
  include_directories(SYSTEM ${VXL_CORE_INCLUDE_DIR})
  link_directories(${VXL_LIBRARY_DIR})
endif( MAPTK_ENABLE_VXL )

if( MAPTK_ENABLE_VISCL )
  find_package(viscl REQUIRED)
  include_directories(SYSTEM ${viscl_INCLUDE_DIR})
endif( MAPTK_ENABLE_VISCL )

if(WIN32)
  set(Boost_USE_STATIC_LIBS True)
endif()
find_package( Boost 1.42 REQUIRED COMPONENTS thread filesystem system date_time regex)
add_definitions( -DBOOST_ALL_NO_LIB )
include_directories(SYSTEM ${Boost_INCLUDE_DIRS})

# this is where the main source code lives
add_subdirectory( maptk )

option( MAPTK_BUILD_TESTS "Build tests" "ON" )
if( MAPTK_BUILD_TESTS )
  enable_testing()
  add_subdirectory( tests )
endif( MAPTK_BUILD_TESTS )

# Configure the CMake EXPORT file during installation
install( FILES ${maptk_all_BINARY_DIR}/CMakeFiles/maptk-config.cmake
  DESTINATION lib/cmake
)
install( EXPORT maptk DESTINATION lib/cmake FILE maptk-config-targets.cmake )


# build a CPack driven installer package
set(CPACK_SYSTEM_NAME "${CMAKE_SYSTEM_NAME}-${CMAKE_SYSTEM_PROCESSOR}")
set(CPACK_PACKAGE_NAME          "maptk")
set(CPACK_PACKAGE_VENDOR        "Kitware, Inc.")
set(CPACK_PACKAGE_CONTACT       "kitware@kitware.com")
set(CPACK_MONOLITHIC_INSTALL    true)
set(CPACK_PACKAGE_VERSION_MAJOR "${maptk_VERSION_MAJOR}")
set(CPACK_PACKAGE_VERSION_MINOR "${maptk_VERSION_MINOR}")
set(CPACK_PACKAGE_VERSION_PATCH "${maptk_VERSION_PATCH}")
set(CPACK_PACKAGE_VERSION       "${maptk_VERSION}")
set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/KITWARE_LICENSE.txt")
include (CPack)
