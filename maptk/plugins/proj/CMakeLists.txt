# maptk/proj

set(maptk_proj_headers_public
  geo_map.h
  proj_config.h
  )

set(maptk_proj_sources
  algorithm_impl_pl_interface.cxx
  geo_map.cxx
  )

maptk_install_plugin_headers( proj
  ${maptk_proj_headers_public}
  )
maptk_add_plugin( proj
  MAKE_PLUGIN_PROJ
  ${maptk_proj_sources}
  )
maptk_plugin_link_libs( proj
  ${PROJ_LIBRARY}
  )
