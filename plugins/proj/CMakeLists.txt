# maptk/proj

set(maptk_proj_headers_public
  geo_map.h
  proj_config.h
  register.h
  )

set(maptk_proj_sources
  geo_map.cxx
  register.cxx
  )

maptk_install_headers(
  ${maptk_proj_headers_public}
  SUBDIR proj
  )

maptk_add_library(maptk_proj
  ${maptk_proj_headers_public}
  ${maptk_proj_sources}
  )
target_link_libraries(maptk_proj
  maptk_core
  ${PROJ_LIBRARY}
  )
