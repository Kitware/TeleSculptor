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
  matrix.h
  ocv_algo_tools.h
  ocv_config.h
  )

set( maptk_ocv_sources
  algorithm_impl_pl_interface.cxx

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
  )

maptk_install_plugin_headers( ocv
  ${maptk_ocv_headers_public}
  )
maptk_add_plugin( ocv
  MAKE_PLUGIN_OCV
  ${maptk_ocv_sources}
  )
maptk_plugin_link_libs( ocv
  ${OpenCV_LIBS}
  )
