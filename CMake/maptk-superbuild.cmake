# Central location for MAPTK external dependency declaration and resolution
include(ExternalProject)

###
# Option for CUDA/CUDNN
#
option(MAPTK_BUILD_WITH_CUDA "Build with CUDA support" FALSE)
if (MAPTK_BUILD_WITH_CUDA)
  find_package( CUDA QUIET REQUIRED )
# Option for CUDNN
#  option(MAPTK_BUILD_WITH_CUDNN "Build with CUDNN support" FALSE)
#  if (MAPTK_BUILD_WITH_CUDNN)
#    set( CUDNN_ROOT_DIR "" CACHE PATH "CUDNN root folder" )
#    find_package( CUDNN QUIET REQUIRED)
#  endif()
#elseif(MAPTK_BUILD_WITH_CUDNN)
#  unset(MAPTK_BUILD_WITH_CUDNN CACHE)
#  message(WARNING "Disabling MAPTK_BUILD_WITH_CUDNN, You must have MAPTK_BUILD_WITH_CUDA enabled for this to be enabled")
endif()

set( kwiver_DIR "" CACHE PATH "Path to KWIVER" )

set( kwiver_FOUND FALSE)
if(kwiver_DIR)
  ## Make sure this is a good kwiver dir
  if ( IS_DIRECTORY ${kwiver_DIR} )
    message(STATUS "Looking for your kwiver in ${kwiver_DIR}...")
    # if we find it, we want to use the fletch_DIR used by kwiver
    # so cache off what we have, and unset it, so we get it in our scope
    find_package( kwiver NO_MODULE NO_POLICY_SCOPE)
    if ( kwiver_FOUND )
      message(STATUS "I found your kwiver!")
      # See if KWIVER has fletch
      if ( IS_DIRECTORY ${fletch_DIR} )
        set( fletch_FOUND TRUE)
        message(STATUS "kwiver uses this fletch ${fletch_DIR}")
        find_package(fletch NO_MODULE)
      else()
        set( fletch_FOUND FALSE)
        message(STATUS "kwiver was not built with fletch")        
      endif()
    else()
      message(STATUS "I could not find your kwiver!")
      set( kwiver_FOUND FALSE)
    endif()
  else()
    message(STATUS "I could not find your kwiver!")
    set( kwiver_FOUND FALSE)
  endif()
endif()

if(NOT kwiver_FOUND)
  message(STATUS "Performing super build!")
  include(CMake/maptk-external-fletch.cmake)
  include(CMake/maptk-external-kwiver.cmake)  
endif()

if(MAPTK_ENABLE_GUI)
  include(CMake/maptk-external-qt-extensions.cmake)
endif()

# MAPTK
set(MAPTK_INNER_DIR ${MAPTK_BINARY_DIR}/MAPTK-build)

ExternalProject_Add(MAPTK
  PREFIX MAPTK
  DEPENDS ${MAPTK_DEPENDENCIES}
  SOURCE_DIR ${MAPTK_SOURCE_DIR}
  BINARY_DIR ${MAPTK_INNER_DIR}
  STAMP_DIR ${MAPTK_STAMP_DIR}
  CMAKE_CACHE_ARGS
    -Dkwiver_DIR:PATH=${kwiver_DIR}
    -DqtExtensions_DIR:PATH=${qtExtensions_DIR}
    -DBUILD_SHARED_LIBS:BOOL=ON
    -DMAPTK_SUPERBUILD:BOOL=OFF
    -DMAPTK_ENABLE_GUI:BOOL=${MAPTK_ENABLE_GUI}
    -DMAPTK_ENABLE_TESTING:BOOL=${MAPTK_ENABLE_TESTING}
    -DCMAKE_PREFIX_PATH:STRING=${CMAKE_PREFIX_PATH}
#    -DCMAKE_INSTALL_PREFIX:STRING=${CMAKE_INSTALL_PREFIX}
    -DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}
    -DCMAKE_CXX_COMPILER:FILEPATH=${CMAKE_CXX_COMPILER}
    -DCMAKE_CXX_FLAGS:STRING=${ep_common_cxx_flags}
    -DCMAKE_C_COMPILER:FILEPATH=${CMAKE_C_COMPILER}
    -DCMAKE_C_FLAGS:STRING=${ep_common_c_flags}
     ${CMAKE_CXX_COMPILER_LAUNCHER_FLAG}
     ${CMAKE_C_COMPILER_LAUNCHER_FLAG}
    -DCMAKE_EXE_LINKER_FLAGS:STRING=${CMAKE_EXE_LINKER_FLAGS}
    -DCMAKE_SHARED_LINKER_FLAGS:STRING=${CMAKE_SHARED_LINKER_FLAGS}
    -DADDITIONAL_C_FLAGS:STRING=${ADDITIONAL_C_FLAGS}
    -DADDITIONAL_CXX_FLAGS:STRING=${ADDITIONAL_CXX_FLAGS}
  INSTALL_COMMAND cmake -E echo "Skipping install step."
  #INSTALL_DIR "${CMAKE_INSTALL_PREFIX}"
)

set(LIBRARY_PATH_VAR "LD_LIBRARY_PATH")
if( APPLE )
  set(LIBRARY_PATH_VAR "DYLD_FALLBACK_LIBRARY_PATH")
endif()
if(WIN32)
  configure_file(${MAPTK_CMAKE_DIR}/start_MSVC.bat.in           ${MAPTK_BINARY_DIR}/start_MSVC.bat @ONLY)
  configure_file(${MAPTK_CMAKE_DIR}/setup_MAPTK.build.bat.in    ${MAPTK_BINARY_DIR}/setup_MAPTK.bat @ONLY)
else()
  configure_file(${MAPTK_CMAKE_DIR}/setup_MAPTK.build.sh.in     ${MAPTK_BINARY_DIR}/setup_MAPTK.sh @ONLY)
endif()
