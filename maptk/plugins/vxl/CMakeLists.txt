# Build / Install plugin containing VXL algorithm implementations + support
# structures

set(maptk_vxl_headers_public
  bundle_adjust.h
  camera.h
  camera_map.h
  close_loops_homography_guided.h
  compute_homography_overlap.h
  estimate_essential_matrix.h
  estimate_homography.h
  estimate_similarity_transform.h
  image_container.h
  image_io.h
  match_features_constrained.h
  optimize_cameras.h
  register_algorithms.h
  triangulate_landmarks.h
  vil_image_memory.h
  )
kwiver_install_headers(
  SUBDIR     maptk/plugins/vxl
  ${maptk_vxl_headers_public}
  )
kwiver_install_headers(
  ${CMAKE_CURRENT_BINARY_DIR}/maptk_vxl_export.h
  NOPATH   SUBDIR     maptk/plugins/vxl
  )

set(maptk_vxl_sources
  bundle_adjust.cxx
  camera.cxx
  camera_map.cxx
  close_loops_homography_guided.cxx
  compute_homography_overlap.cxx
  estimate_essential_matrix.cxx
  estimate_homography.cxx
  estimate_similarity_transform.cxx
  image_container.cxx
  image_io.cxx
  match_features_constrained.cxx
  optimize_cameras.cxx
  register_algorithms.cxx
  triangulate_landmarks.cxx
  vil_image_memory.cxx
  )

kwiver_add_library( maptk_vxl
  ${maptk_vxl_headers_public}
  ${maptk_vxl_sources}
  )
target_link_libraries( maptk_vxl
  PUBLIC               maptk
                       rrel rsdl vnl vgl_algo vil vpgl_algo
  )

maptk_create_plugin( maptk_vxl
  algorithm_plugin_interface.cxx
  )
