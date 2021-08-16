include(CheckCXXCompilerFlag)

include_directories("${TELESCULPTOR_SOURCE_DIR}")
include_directories("${TELESCULPTOR_BINARY_DIR}")


###
# Setting up main library
#
set(maptk_public_headers
  geo_reference_points_io.h
  ground_control_point.h
  write_pdal.h
  )

set(maptk_private_headers
  "${CMAKE_CURRENT_BINARY_DIR}/version.h"
  "${CMAKE_CURRENT_BINARY_DIR}/maptk_config.h"
  )

set(maptk_sources
  geo_reference_points_io.cxx
  ground_control_point.cxx
  write_pdal.cxx
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

option(TELESCULPTOR_USE_PDAL "Enable PDAL support for saving to LAS" ON)
if(TELESCULPTOR_USE_PDAL)
  find_package(PDAL 1.0.0 REQUIRED)

  # PDAL library names are improperly exported in PDAL 1.7.2 for Linux
  if (${PDAL_VERSION} VERSION_EQUAL "1.7.2" AND NOT (WIN32 OR APPLE))
    set(PDAL_LIBRARIES "pdal_base")
  endif()

  target_link_libraries( maptk
    PRIVATE              ${PDAL_LIBRARIES}
    )
  target_include_directories( maptk SYSTEM
    PRIVATE                   ${PDAL_INCLUDE_DIRS}
                              ${PDAL_INCLUDE_DIRS}/pdal
    )
endif()

configure_file("${CMAKE_CURRENT_SOURCE_DIR}/maptk_config.h.in"
               "${CMAKE_CURRENT_BINARY_DIR}/maptk_config.h")

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
