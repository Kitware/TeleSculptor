# Build / Install plugin containing Ceres algorithm implementations + support
# structures

set( maptk_ceres_headers_public
  bundle_adjust.h
  ceres_config.h
  lens_distortion.h
  register_algorithms.h
  reprojection_error.h
  )
set( maptk_ceres_headers_private
  types.h
  )
maptk_install_plugin_headers( ceres
  ${maptk_ceres_headers_public}
  )

set( maptk_ceres_sources
  bundle_adjust.cxx
  register_algorithms.cxx
  types.cxx
  )
maptk_add_library( maptk_ceres
  ${maptk_ceres_headers_public}
  ${maptk_ceres_headers_private}
  ${maptk_ceres_sources}
  )
target_link_libraries( maptk_ceres
  maptk
  ceres
  )

maptk_create_plugin( maptk_ceres
  algorithm_plugin_interface.cxx
  BUNDLE_PATHS  $<TARGET_FILE_DIR:ceres>
  )
