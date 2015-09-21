# maptk/proj

set(maptk_proj_headers_public
  geo_map.h
  proj_config.h
  register_algorithms.h
  )
kwiver_install_headers(
  SUBDIR     maptk/plugins/proj
  ${maptk_proj_headers_public}
  )

set(maptk_proj_sources
  geo_map.cxx
  register_algorithms.cxx
  )
maptk_add_library( maptk_proj
  ${maptk_proj_headers_public}
  ${maptk_proj_sources}
  )
target_link_libraries( maptk_proj
  maptk
  ${PROJ_LIBRARY}
  )
maptk_create_plugin( maptk_proj
  algorithm_plugin_interface.cxx
  )
