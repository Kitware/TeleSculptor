# Build / Install Plugin containing default algorithm implementations

set( plugin_default_headers
  close_loops_bad_frames_only.h
  close_loops_multi_method.h
  compute_ref_homography_default.h
  convert_image_default.h
  hierarchical_bundle_adjust.h
  match_features_homography.h
  track_features_default.h
  )

set( plugin_default_sources
  algorithm_impl_pl_interface.cxx

  close_loops_bad_frames_only.cxx
  close_loops_multi_method.cxx
  compute_ref_homography_default.cxx
  convert_image_default.cxx
  hierarchical_bundle_adjust.cxx
  match_features_homography.cxx
  track_features_default.cxx
  )

maptk_install_plugin_headers( default
  ${plugin_default_headers}
  )
maptk_add_plugin( default
  MAKE_PLUGIN_DEFAULT
  ${plugin_default_sources}
  )
