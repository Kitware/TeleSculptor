include(CheckCXXCompilerFlag)

include_directories("${TELESCULPTOR_SOURCE_DIR}")
include_directories("${TELESCULPTOR_BINARY_DIR}")


###
# Setting up main library
#
set(maptk_public_headers
  geo_reference_points_io.h
  ground_control_point.h
  )

set(maptk_private_headers
  "${CMAKE_CURRENT_BINARY_DIR}/version.h"
  )

set(maptk_sources
  geo_reference_points_io.cxx
  ground_control_point.cxx
  )

kwiver_configure_file( version.h
  "${CMAKE_CURRENT_SOURCE_DIR}/version.h.in"
  "${CMAKE_CURRENT_BINARY_DIR}/version.h"
  TeleSculptor_VERSION
  TELESCULPTOR_COPYRIGHT_YEAR
)

kwiver_add_library(
  maptk
  ${maptk_public_headers}
  ${maptk_private_headers}
  ${maptk_sources}
  )

target_link_libraries( maptk
  PUBLIC               kwiver::vital
                       kwiver::kwiversys
  )

# Configuring/Adding compile definitions to target
# (so we can use generator expressions)

if (CMAKE_GENERATOR_IS_MULTI_CONFIG)
  set(config_subdir "/$<CONFIGURATION>")
endif()

###
# Marking source root for documentation generation
#
if(NOT DOXYGEN_TELESCULPTOR_NUMBER)
  set(DOXYGEN_TELESCULPTOR_NUMBER "v${TeleSculptor_VERSION}")
endif()
kwiver_create_doxygen( maptk "${CMAKE_CURRENT_LIST_DIR}"
                       DISPLAY_NAME "MAP-Tk"
                       VERSION_NUMBER "${DOXYGEN_TELESCULPTOR_NUMBER}")
