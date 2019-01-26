include(CheckCXXCompilerFlag)

include_directories("${MAPTK_SOURCE_DIR}")
include_directories("${MAPTK_BINARY_DIR}")

###
# Setting up main library
#
set(maptk_public_headers
  geo_reference_points_io.h
  ground_control_point.h
  )

set(maptk_private_headers
  colorize.h
  "${CMAKE_CURRENT_BINARY_DIR}/version.h"
  )

set(maptk_sources
  colorize.cxx
  geo_reference_points_io.cxx
  ground_control_point.cxx
  )

kwiver_configure_file( version.h
  "${CMAKE_CURRENT_SOURCE_DIR}/version.h.in"
  "${CMAKE_CURRENT_BINARY_DIR}/version.h"
  MAPTK_VERSION
  MAPTK_COPYRIGHT_YEAR
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

if (CMAKE_CONFIGURATION_TYPES)
  set(config_subdir "/$<CONFIGURATION>")
endif()

###
# Marking source root for documentation generation
#
if(NOT DOXYGEN_MAPTK_NUMBER)
  set(DOXYGEN_MAPTK_NUMBER "v${MAPTK_VERSION}")
endif()
kwiver_create_doxygen( maptk "${CMAKE_CURRENT_LIST_DIR}"
                       DISPLAY_NAME "MAP-Tk"
                       VERSION_NUMBER "${DOXYGEN_MAPTK_NUMBER}")
