# maptk/core

set(maptk_core_headers_public
  camera.h
  camera_intrinsics.h
  config.h
  config_io.h
  core-config.h
  covariance.h
  descriptor.h
  descriptor_set.h
  exceptions.h
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
  types.h
  vector.h
  vector_cmath.h

  exceptions/base.h
  exceptions/config.h
  exceptions/config_io.h

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
  config.cxx
  config_io.cxx
  feature.cxx
  image.cxx
  ins_data.cxx
  matrix.cxx
  rotation.cxx
  track.cxx
  track_set.cxx
  vector.cxx

  exceptions/base.cxx
  exceptions/config.cxx
  exceptions/config_io.cxx

  algo/detect_features.cxx
  algo/estimate_homography.cxx
  algo/extract_descriptors.cxx
  algo/geo_map.cxx
  algo/image_io.cxx
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
target_link_libraries(maptk_core
  ${Boost_LIBRARIES}
  )
set_target_properties(maptk_core
  PROPERTIES
    VERSION       ${MAPTK_VERSION}
    SOVERSION     0
    DEFINE_SYMBOL MAKE_MAPTK_CORE_LIB
  )
