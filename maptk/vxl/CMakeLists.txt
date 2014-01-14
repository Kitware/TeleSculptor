# maptk/vxl

include_directories( SYSTEM ${VXL_RPL_INCLUDE_DIR} )

set(maptk_vxl_headers_public
  camera.h
  estimate_homography.h
  image_container.h
  image_io.h
  register.h
  vil_image_memory.h
  vxl_config.h
  )

set(maptk_vxl_sources
  camera.cxx
  estimate_homography.cxx
  image_container.cxx
  image_io.cxx
  register.cxx
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

target_link_libraries(maptk_vxl maptk_core vil vpgl rrel)
