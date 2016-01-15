# Build / Install Plugin containing core algorithm implementations

set( plugin_core_headers
  close_loops_bad_frames_only.h
  close_loops_multi_method.h
  compute_ref_homography_core.h
  convert_image_bypass.h
  filter_features_magnitude.h
  hierarchical_bundle_adjust.h
  initialize_cameras_landmarks.h
  match_features_homography.h
  plugin_core_config.h
  register_algorithms.h
  track_features_core.h
  triangulate_landmarks.h
  )
maptk_install_plugin_headers( core
  ${plugin_core_headers}
  )

set( plugin_core_sources
  close_loops_bad_frames_only.cxx
  close_loops_multi_method.cxx
  compute_ref_homography_core.cxx
  convert_image_bypass.cxx
  filter_features_magnitude.cxx
  hierarchical_bundle_adjust.cxx
  initialize_cameras_landmarks.cxx
  match_features_homography.cxx
  register_algorithms.cxx
  track_features_core.cxx
  triangulate_landmarks.cxx
  )
maptk_add_library( maptk_core
  ${plugin_core_headers}
  ${plugin_core_sources}
  )
target_link_libraries( maptk_core maptk )

get_property(libd TARGET maptk PROPERTY LOCATION)
get_filename_component(LIBRARY_DIR ${libd} DIRECTORY)

maptk_create_plugin( maptk_core
  algorithm_plugin_interface.cxx
  BUNDLE_PATHS  ${LIBRARY_DIR}
  )
