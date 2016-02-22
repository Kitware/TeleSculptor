# Build / Install plugin containing Ceres algorithm implementations + support
# structures

set( maptk_ceres_headers_public
  bundle_adjust.h
  lens_distortion.h
  register_algorithms.h
  reprojection_error.h
  )
set( maptk_ceres_headers_private
  types.h
  )
kwiver_install_headers(
  SUBDIR     maptk/plugins/ceres
  ${maptk_ceres_headers_public}
  )
kwiver_install_headers(
  ${CMAKE_CURRENT_BINARY_DIR}/maptk_ceres_export.h
  NOPATH   SUBDIR     maptk/plugins/ceres
  )

set( maptk_ceres_sources
  bundle_adjust.cxx
  register_algorithms.cxx
  types.cxx
  )
kwiver_add_library( maptk_ceres

  ${maptk_ceres_headers_public}
  ${maptk_ceres_headers_private}
  ${maptk_ceres_sources}
  )
target_link_libraries( maptk_ceres
  PUBLIC               maptk
                       ceres
  )

maptk_create_plugin( maptk_ceres
  algorithm_plugin_interface.cxx
  )
