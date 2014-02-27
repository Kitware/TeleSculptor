# maptk/vxl

include_directories( SYSTEM ${VXL_RPL_INCLUDE_DIR} )

set(maptk_vxl_headers_public
  bundle_adjust.h
  camera.h
  camera_map.h
  estimate_homography.h
  estimate_similarity_transform.h
  image_container.h
  image_io.h
  register.h
  triangulate_landmarks.h
  vil_image_memory.h
  vxl_config.h
  )

set(maptk_vxl_sources
  bundle_adjust.cxx
  camera.cxx
  camera_map.cxx
  estimate_homography.cxx
  estimate_similarity_transform.cxx
  image_container.cxx
  image_io.cxx
  register.cxx
  triangulate_landmarks.cxx
  vil_image_memory.cxx
  )

# installing header files
maptk_install_headers(
  ${maptk_vxl_headers_public}
  SUBDIR vxl
  )

maptk_add_library(maptk_vxl
  ${maptk_vxl_headers_public}
  ${maptk_vxl_sources}
  )

target_link_libraries(maptk_vxl maptk_core vil vpgl_algo rrel vgl_algo)
