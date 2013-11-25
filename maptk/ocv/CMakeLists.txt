# maptk/ocv

set(maptk_ocv_headers_public
  descriptor_set.h
  detect_features.h
  extract_descriptors.h
  feature_set.h
  image_container.h
  mat_image_memory.h
  image_io.h
  )

set(maptk_ocv_sources
  descriptor_set.cxx
  detect_features.cxx
  extract_descriptors.cxx
  feature_set.cxx
  image_container.cxx
  mat_image_memory.cxx
  image_io.cxx
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
