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
  initialize_cameras_landmarks.h
  optimize_cameras.h
  triangulate_landmarks.h
  vil_image_memory.h
  vxl_config.h
  )

set(maptk_vxl_sources
  algorithm_impl_pl_interface.cxx

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
  initialize_cameras_landmarks.cxx
  optimize_cameras.cxx
  triangulate_landmarks.cxx
  vil_image_memory.cxx
  )

# installing header files
maptk_install_plugin_headers( vxl
  ${maptk_vxl_headers_public}
  )
maptk_add_plugin( vxl
  MAKE_PLUGIN_VXL
  #${maptk_vxl_headers_public}
  ${maptk_vxl_sources}
  )
maptk_plugin_link_libs( vxl
  rrel vgl_algo vil vpgl_algo
  )
