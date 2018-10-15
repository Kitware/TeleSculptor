# Central location for MAPTK external dependency declaration and resolution
include(ExternalProject)

###
# Option for CUDA
#
option(MAPTK_BUILD_WITH_CUDA "Build with CUDA support" FALSE)
if (MAPTK_BUILD_WITH_CUDA)
  find_package( CUDA QUIET REQUIRED )
endif()

set( kwiver_DIR "" CACHE PATH "Path to KWIVER" )

set( kwiver_FOUND FALSE)
if(kwiver_DIR)
  ## Make sure this is a good kwiver dir
  if ( IS_DIRECTORY ${kwiver_DIR} )
    message(STATUS "Looking for kwiver in ${kwiver_DIR}...")
    # if we find it, we want to use the fletch_DIR used by kwiver
    find_package( kwiver NO_MODULE NO_POLICY_SCOPE)
    if ( kwiver_FOUND )
      message(STATUS "I found kwiver!")
      # See if KWIVER has fletch
      if ( IS_DIRECTORY ${fletch_DIR} )
        set( fletch_FOUND TRUE)
        message(STATUS "kwiver uses this fletch : ${fletch_DIR}")
        find_package(fletch NO_MODULE)
      else()
        set( fletch_FOUND FALSE)
        message(STATUS "kwiver was not built with fletch")        
      endif()
    else()
      message(STATUS "Could not find your kwiver!")
      set( kwiver_FOUND FALSE)
    endif()
  else()
    message(STATUS "Could not find your kwiver!")
    set( kwiver_FOUND FALSE)
  endif()
endif()


if(NOT kwiver_FOUND)
  message(STATUS "Performing super build!")
  include(CMake/maptk-external-fletch.cmake)
  include(CMake/maptk-external-kwiver.cmake)
endif()

# MAPTK

ExternalProject_Add(maptk
  PREFIX ${MAPTK_BINARY_DIR}
  DEPENDS ${MAPTK_DEPENDENCIES}
  SOURCE_DIR ${MAPTK_SOURCE_DIR}
  BINARY_DIR ${MAPTK_EXTERNAL_DIR}/maptk-build
  STAMP_DIR ${MAPTK_STAMP_DIR}
  CMAKE_CACHE_ARGS
    -Dkwiver_DIR:PATH=${kwiver_DIR}
    -DqtExtensions_DIR:PATH=${qtExtensions_DIR}
    -DBUILD_SHARED_LIBS:BOOL=ON
    -DMAPTK_SUPERBUILD:BOOL=OFF
    -DMAPTK_ENABLE_GUI:BOOL=${MAPTK_ENABLE_GUI}
    -DMAPTK_ENABLE_TESTING:BOOL=${MAPTK_ENABLE_TESTING}
    -DMAPTK_FIXUP_BUNDLE_ON_PACKAGE:BOOL=ON
    -DCMAKE_PREFIX_PATH:STRING=${CMAKE_PREFIX_PATH}
    -DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}
    -DCMAKE_CONFIGURATION_TYPES:STRING=${CMAKE_CONFIGURATION_TYPES}
    -DCMAKE_CXX_COMPILER:FILEPATH=${CMAKE_CXX_COMPILER}
    -DCMAKE_CXX_FLAGS:STRING=${CMAKE_CXX_FLAGS}
    -DCMAKE_C_COMPILER:FILEPATH=${CMAKE_C_COMPILER}
    -DCMAKE_C_FLAGS:STRING=${ep_common_c_flags}
     ${CMAKE_CXX_COMPILER_LAUNCHER_FLAG}
     ${CMAKE_C_COMPILER_LAUNCHER_FLAG}
    -DCMAKE_EXE_LINKER_FLAGS:STRING=${CMAKE_EXE_LINKER_FLAGS}
    -DCMAKE_SHARED_LINKER_FLAGS:STRING=${CMAKE_SHARED_LINKER_FLAGS}
    -DADDITIONAL_C_FLAGS:STRING=${ADDITIONAL_C_FLAGS}
    -DADDITIONAL_CXX_FLAGS:STRING=${ADDITIONAL_CXX_FLAGS}
  INSTALL_COMMAND cmake -E echo "Skipping install step."
)

