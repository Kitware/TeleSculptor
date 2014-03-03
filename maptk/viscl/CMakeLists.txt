# maptk/viscl

include_directories(SYSTEM ${viscl_OPENCL_INCLUDE_DIRS})

set(maptk_viscl_headers_public
  convert_image.h
  descriptor_set.h
  detect_features.h
  extract_descriptors.h
  feature_set.h
  image_container.h
  match_features.h
  match_set.h
  viscl_config.h
  register.h
  utils.h
  )

set(maptk_viscl_sources
  convert_image.cxx
  descriptor_set.cxx
  detect_features.cxx
  extract_descriptors.cxx
  feature_set.cxx
  image_container.cxx
  match_features.cxx
  match_set.cxx
  register.cxx
  utils.cxx
  )

# installing header files
maptk_install_headers(
  ${maptk_viscl_headers_public}
  SUBDIR viscl
  )

maptk_add_library(maptk_viscl
  ${maptk_viscl_headers_public}
  ${maptk_viscl_sources}
  )

target_link_libraries( maptk_viscl maptk_core viscl_core viscl_tasks viscl_vxl)
