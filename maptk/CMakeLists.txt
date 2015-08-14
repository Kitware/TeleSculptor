include(CheckCXXCompilerFlag)

check_cxx_compiler_flag(-fvisibility=hidden MAPTK_HAVE_GCC_VISIBILITY)


# Configure general config.h.in
maptk_configure_file(config.h
  "${CMAKE_CURRENT_SOURCE_DIR}/config.h.in"
  "${CMAKE_CURRENT_BINARY_DIR}/config.h"
  BUILD_SHARED_LIBS
  MAPTK_HAVE_GCC_VISIBILITY
  )
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/config.h"
  DESTINATION include/maptk
  )


include_directories("${MAPTK_SOURCE_DIR}")
include_directories("${MAPTK_BINARY_DIR}")


###
# Setting up core library
#
set(maptk_public_headers
  metrics.h

  local_geo_cs.h
  projected_track_set.h
  geo_reference_points_io.h
  ins_data.h
  ins_data_io.h
  match_matrix.h
  triangulate.h

  plugin_interface/algorithm_plugin_interface.h
  plugin_interface/algorithm_plugin_interface_macros.h
  )


set(maptk_sources
  metrics.cxx

  local_geo_cs.cxx
  projected_track_set.cxx
  geo_reference_points_io.cxx
  ins_data.cxx
  ins_data_io.cxx
  match_matrix.cxx
  triangulate.cxx

  )

maptk_install_headers(
  ${maptk_public_headers}
  )
maptk_add_library(
  maptk
  ${maptk_public_headers}
  ${maptk_sources}
  )
target_link_libraries(maptk vital ${Boost_LIBRARIES} )


###
#Cconfigure plugins
#
add_subdirectory(plugins)


# Configuring/Adding compile definitions to target
# (so we can use generator expressions)

if (CMAKE_CONFIGURATION_TYPES)
  set(config_subdir "/$<CONFIGURATION>")
endif()

###
# Marking source root for documentation generation
#
maptk_create_doxygen( maptk "${CMAKE_CURRENT_LIST_DIR}" "MAP-Tk")
