# kwiver External Project

message(STATUS "Configuring external KWIVER")

list(APPEND TELESCULPTOR_DEPENDENCIES kwiver)

# For now, OpenMP is not properly supported on macOS
if(APPLE)
  set(MAPTK_ENABLE_OPENMP OFF)
else()
  set(MAPTK_ENABLE_OPENMP ON)
endif()

ExternalProject_Add(kwiver
  DEPENDS ${KWIVER_DEPENDENCIES}
  PREFIX ${TELESCULPTOR_BINARY_DIR}
  SOURCE_DIR ${TELESCULPTOR_EXTERNAL_DIR}/kwiver
  BINARY_DIR ${TELESCULPTOR_EXTERNAL_DIR}/kwiver-build
  STAMP_DIR ${TELESCULPTOR_STAMP_DIR}
  GIT_REPOSITORY "https://github.com/Kitware/kwiver.git"
  GIT_TAG 1d47113490f123e89286e721fcf2d49aaed81ad2
  #GIT_SHALLOW 1
  CMAKE_CACHE_ARGS
    -DBUILD_SHARED_LIBS:BOOL=ON
    -Dfletch_DIR:PATH=${fletch_DIR}
    -DKWIVER_ENABLE_ARROWS:BOOL=ON
    -DKWIVER_ENABLE_BURNOUT:BOOL=OFF
    -DKWIVER_ENABLE_CERES:BOOL=ON
    -DKWIVER_ENABLE_CUDA:BOOL=${TELESCULPTOR_ENABLE_CUDA}
    -DKWIVER_ENABLE_C_BINDINGS:BOOL=${TELESCULPTOR_ENABLE_PYTHON}
    -DKWIVER_ENABLE_DARKNET:BOOL=OFF
    -DKWIVER_ENABLE_DLL_WARNINGS:BOOL=OFF
    -DKWIVER_ENABLE_DOCS:BOOL=OFF
    -DKWIVER_ENABLE_EXAMPLES:BOOL=OFF
    -DKWIVER_ENABLE_EXTRAS:BOOL=ON
    -DKWIVER_ENABLE_FFMPEG:BOOL=ON
    -DKWIVER_ENABLE_KPF:BOOL=OFF
    -DKWIVER_ENABLE_GDAL:BOOL=ON
    -DKWIVER_ENABLE_LOG4CPLUS:BOOL=ON
    -DKWIVER_ENABLE_LOG4CXX:BOOL=OFF
    -DKWIVER_ENABLE_MATLAB:BOOL=OFF
    -DKWIVER_ENABLE_MVG:BOOL=ON
    -DKWIVER_ENABLE_OPENCV:BOOL=ON
    -DKWIVER_ENABLE_OPENMP:BOOL=${MAPTK_ENABLE_OPENMP}
    -DKWIVER_ENABLE_PROCESSES:BOOL=ON
    -DKWIVER_ENABLE_PROJ:BOOL=ON
    -DKWIVER_ENABLE_PYTHON:BOOL=${TELESCULPTOR_ENABLE_PYTHON}
    -DKWIVER_ENABLE_QT:BOOL=ON
    -DKWIVER_ENABLE_QT_EXT:BOOL=ON
    -DKWIVER_ENABLE_RightTrack:BOOL=OFF
    -DKWIVER_ENABLE_SPROKIT:BOOL=ON
    -DKWIVER_ENABLE_SUPER3D:BOOL=ON
    -DKWIVER_ENABLE_TESTS:BOOL=${TELESCULPTOR_ENABLE_TESTING}
    -DKWIVER_ENABLE_TOOLS:BOOL=${TELESCULPTOR_ENABLE_TOOLS}
    -DKWIVER_ENABLE_TRACK_ORACLE:BOOL=OFF
    -DKWIVER_ENABLE_UUID:BOOL=OFF
    -DKWIVER_ENABLE_VISCL:BOOL=OFF
    -DKWIVER_ENABLE_VTK:BOOL=ON
    -DKWIVER_ENABLE_VXL:BOOL=ON
    -DKWIVER_INSTALL_SET_UP_SCRIPT:BOOL=ON
    -DKWIVER_TEST_ADD_TARGETS:BOOL=OFF
    -DKWIVER_USE_BUILD_TREE:BOOL=OFF
    -DKWIVER_USE_CONFIGURATION_SUBDIRECTORY:BOOL=OFF
    -DCMAKE_PREFIX_PATH:STRING=${CMAKE_PREFIX_PATH}
    -DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}
    -DCMAKE_BUILD_WITH_INSTALL_RPATH:BOOL=ON
    -DCMAKE_CONFIGURATION_TYPES:STRING=${CMAKE_CONFIGURATION_TYPES}
    -DCMAKE_CXX_COMPILER:FILEPATH=${CMAKE_CXX_COMPILER}
    -DCMAKE_CXX_FLAGS:STRING=${CMAKE_CXX_FLAGS}
    -DCMAKE_C_COMPILER:FILEPATH=${CMAKE_C_COMPILER}
    -DCMAKE_C_FLAGS:STRING=${CMAKE_C_FLAGS}
    ${CMAKE_CXX_COMPILER_LAUNCHER_FLAG}
    ${CMAKE_C_COMPILER_LAUNCHER_FLAG}
    -DCMAKE_EXE_LINKER_FLAGS:STRING=${CMAKE_EXE_LINKER_FLAGS}
    -DCMAKE_SHARED_LINKER_FLAGS:STRING=${CMAKE_SHARED_LINKER_FLAGS}
    -DMAKECOMMAND:STRING=${MAKECOMMAND}
    -DPYTHON_VERSION_MAJOR:STRING=3
    -DADDITIONAL_C_FLAGS:STRING=${ADDITIONAL_C_FLAGS}
    -DADDITIONAL_CXX_FLAGS:STRING=${ADDITIONAL_CXX_FLAGS}
  INSTALL_COMMAND ${CMAKE_COMMAND} -E echo "Skipping install step."
  USES_TERMINAL_BUILD 1
)

set(KWIVER_DIR "${TELESCULPTOR_EXTERNAL_DIR}/kwiver-build")
set(kwiver_DIR "${TELESCULPTOR_EXTERNAL_DIR}/kwiver-build" CACHE PATH "Location of KWIVER" FORCE)
