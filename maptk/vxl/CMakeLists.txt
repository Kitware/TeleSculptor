# maptk/vxl

set(maptk_vxl_headers_public
  image_container.h
  vil_image_memory.h
  image_io.h
  register.h
  )

set(maptk_vxl_sources
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

target_link_libraries(maptk_vxl maptk_core vil)
