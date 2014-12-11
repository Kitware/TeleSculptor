# Build / Install Plugin containing default algorithm implementations

set( plugin_default_headers
  close_loops_bad_frames_only.h
  close_loops_multi_method.h
  compute_ref_homography_default.h
  convert_image_default.h
  hierarchical_bundle_adjust.h
  match_features_homography.h
  register_algorithms.h
  track_features_default.h
  )
maptk_install_plugin_headers( default
  ${plugin_default_headers}
  )

set( plugin_default_sources
  close_loops_bad_frames_only.cxx
  close_loops_multi_method.cxx
  compute_ref_homography_default.cxx
  convert_image_default.cxx
  hierarchical_bundle_adjust.cxx
  match_features_homography.cxx
  register_algorithms.cxx
  track_features_default.cxx
  )
maptk_add_library( maptk_default
  ${plugin_default_sources}
  )
target_link_libraries( maptk_default maptk )

maptk_create_plugin( maptk_default
  algorithm_plugin_interface.cxx
  )
