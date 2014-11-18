
set(plugin_default_sources
  close_loops_bad_frames_only.h       close_loops_bad_frames_only.cxx
  close_loops_multi_method.h          close_loops_multi_method.cxx
  compute_ref_homography_default.h    compute_ref_homography_default.cxx
  hierarchical_bundle_adjust.h        hierarchical_bundle_adjust.cxx
  match_features_homography.h         match_features_homography.cxx
  track_features_default.h            track_features_default.cxx

  algorithm_impl_pl_interface.cxx
  )

maptk_add_plugin(default MAKE_PLUGIN_DEFAULT
  ${plugin_default_sources}
  )
target_link_libraries(default
  maptk
  )
