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
  match_set.h
  matrix.h
  rotation.h
  registrar.h
  track.h
  track_set.h
  vector.h
  vector_cmath.h

  algo/algorithm.h
  algo/algorithm.txx
  algo/detect_features.h
  algo/estimate_homography.h
  algo/extract_descriptors.h
  algo/image_io.h
  algo/geo_map.h
  algo/match_features.h
  algo/match_features_homography.h
  algo/register.h
  algo/track_features.h
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
  track_set.cxx
  vector.cxx

  algo/detect_features.cxx
  algo/estimate_homography.cxx
  algo/extract_descriptors.cxx
  algo/image_io.cxx
  algo/geo_map.cxx
  algo/match_features.cxx
  algo/match_features_homography.cxx
  algo/register.cxx
  algo/track_features.cxx
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
