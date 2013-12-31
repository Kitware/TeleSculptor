# maptk/vxl

include_directories( SYSTEM ${VXL_RPL_INCLUDE_DIR} )

set(maptk_vxl_headers_public
  estimate_homography.h
  image_container.h
  vil_image_memory.h
  image_io.h
  register.h
  )

set(maptk_vxl_sources
  estimate_homography.cxx
  image_container.cxx
  vil_image_memory.cxx
  image_io.cxx
  register.cxx
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

target_link_libraries(maptk_vxl maptk_core vil rrel)
