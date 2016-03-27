# Build / Install plugin containing OpenCV algorithm implementations + support
# structures

set( maptk_ocv_headers_public
  analyze_tracks.h
  descriptor_set.h
  detect_features.h
  detect_features_FAST.h
  detect_features_MSER.h
  draw_tracks.h
  estimate_homography.h
  extract_descriptors.h
  extract_descriptors_BRIEF.h
  feature_set.h
  feature_detect_extract_BRISK.h
  feature_detect_extract_ORB.h
  image_container.h
  image_io.h
  mat_image_memory.h
  match_features.h
  match_features_bruteforce.h
  match_features_flannbased.h
  match_set.h
  register_algorithms.h
  )
kwiver_install_headers(
  SUBDIR     maptk/plugins/ocv
  ${maptk_ocv_headers_public}
  )
kwiver_install_headers(
  ${CMAKE_CURRENT_BINARY_DIR}/maptk_ocv_export.h
  NOPATH   SUBDIR     maptk/plugins/ocv
  )

set( maptk_ocv_sources
  analyze_tracks.cxx
  descriptor_set.cxx
  detect_features.cxx
  detect_features_FAST.cxx
  detect_features_MSER.cxx
  draw_tracks.cxx
  estimate_homography.cxx
  extract_descriptors.cxx
  extract_descritpors_BRIEF.cxx
  feature_set.cxx
  feature_detect_extract_BRISK.cxx
  feature_detect_extract_ORB.cxx
  image_container.cxx
  image_io.cxx
  mat_image_memory.cxx
  match_features.cxx
  match_features_bruteforce.cxx
  match_features_flannbased.cxx
  match_set.cxx
  register_algorithms.cxx
  )

if( OpenCV_FOUND AND
    (OpenCV_VERSION VERSION_GREATER "3.0" OR
     OpenCV_VERSION VERSION_EQUAL "3.0") )
  message(STATUS "Found OPENCV 3.0")
  add_definitions(-DMAPTK_HAS_OPENCV_VER_3)
endif()

kwiver_add_library( maptk_ocv
  ${maptk_ocv_headers_public}
  ${maptk_ocv_sources}
  )
target_link_libraries( maptk_ocv
  PUBLIC               maptk
                     ${OpenCV_LIBS}
  )

maptk_create_plugin( maptk_ocv
  algorithm_plugin_interface.cxx
  )
