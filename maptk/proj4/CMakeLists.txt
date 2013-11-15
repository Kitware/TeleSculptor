# maptk/proj4

set(maptk_proj4_headers_public
  geo_map.h
  )

set(maptk_proj4_sources
  geo_map.cxx
  )

maptk_install_headers(
  ${maptk_proj4_headers_public}
  SUBDIR proj4
  )

maptk_add_library(maptk_proj4
  ${maptk_proj4_headers_public}
  ${maptk_proj4_sources}
  )
target_link_libraries(maptk_proj4
  maptk_core
  ${PROJ4_LIBRARY}
  )
