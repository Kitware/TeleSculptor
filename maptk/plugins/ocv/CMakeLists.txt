# Build / Install plugin containing OpenCV algorithm implementations + support
# structures

set( maptk_ocv_headers_public
  analyze_tracks.h
  descriptor_set.h
  detect_features.h
  draw_tracks.h
  estimate_homography.h
  extract_descriptors.h
  feature_set.h
  image_container.h
  image_io.h
  mat_image_memory.h
  match_features.h
  match_set.h
  ocv_algo_tools.h
  ocv_config.h
  register_algorithms.h
  )
maptk_install_plugin_headers( ocv
  ${maptk_ocv_headers_public}
  )

set( maptk_ocv_sources
  analyze_tracks.cxx
  descriptor_set.cxx
  detect_features.cxx
  draw_tracks.cxx
  estimate_homography.cxx
  extract_descriptors.cxx
  feature_set.cxx
  image_container.cxx
  image_io.cxx
  mat_image_memory.cxx
  match_features.cxx
  match_set.cxx
  ocv_algo_tools.cxx
  register_algorithms.cxx
  )
maptk_add_library( maptk_ocv
  ${maptk_ocv_sources}
  )
target_link_libraries( maptk_ocv
  maptk
  ${OpenCV_LIBS}
  )

maptk_create_plugin( maptk_ocv
  algorithm_plugin_interface.cxx
  )
