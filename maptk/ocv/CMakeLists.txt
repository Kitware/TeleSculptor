# maptk/ocv

set(maptk_ocv_headers_public
  analyze_tracks.h
  descriptor_set.h
  detect_features.h
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
  register.h
  )

set(maptk_ocv_sources
  analyze_tracks.cxx
  descriptor_set.cxx
  detect_features.cxx
  estimate_homography.cxx
  extract_descriptors.cxx
  feature_set.cxx
  image_container.cxx
  image_io.cxx
  mat_image_memory.cxx
  match_features.cxx
  match_set.cxx
  ocv_algo_tools.cxx
  register.cxx
  )

# installing header files
maptk_install_headers(
  ${maptk_ocv_headers_public}
  SUBDIR ocv
  )

maptk_add_library(maptk_ocv
  ${maptk_ocv_headers_public}
  ${maptk_ocv_sources}
  )

target_link_libraries( maptk_ocv maptk_core ${OpenCV_LIBS} )
