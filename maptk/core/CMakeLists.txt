# maptk/core

set(maptk_core_headers_public
  camera.h
  camera_intrinsics.h
  covariance.h
  descriptor.h
  descriptor_set.h
  feature.h
  feature_set.h
  image.h
  image_container.h
  ins_data.h
  landmark.h
  matrix.h
  rotation.h
  registrar.h
  track.h
  vector.h
  vector_cmath.h

  algo/algorithm.h
  algo/algorithm.txx
  algo/image_io.h
  algo/detect_features.h
  algo/extract_descriptors.h
  algo/geo_map.h
  )

set(maptk_core_sources
  camera.cxx
  camera_intrinsics.cxx
  feature.cxx
  image.cxx
  ins_data.cxx
  matrix.cxx
  rotation.cxx
  track.cxx
  vector.cxx

  algo/image_io.cxx
  algo/geo_map.cxx
  )

# installing header files
maptk_install_headers(
  ${maptk_core_headers_public}
  SUBDIR core
  )

maptk_add_library(maptk_core
  ${maptk_core_headers_public}
  ${maptk_core_sources}
  )
