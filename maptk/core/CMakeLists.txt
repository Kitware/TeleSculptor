# maptk/core

set(maptk_core_headers_public
  camera.h
  camera_intrinsics.h
  camera_io.h
  camera_map.h
  config_block.h
  config_block_io.h
  core_config.h
  covariance.h
  descriptor.h
  descriptor_set.h
  exceptions.h
  feature.h
  feature_set.h
  image.h
  image_container.h
  ins_data.h
  ins_data_io.h
  landmark.h
  landmark_map.h
  landmark_map_io.h
  local_geo_cs.h
  match_set.h
  matrix.h
  metrics.h
  rotation.h
  registrar.h
  track.h
  track_set.h
  track_set_io.h
  types.h
  vector.h
  vector_cmath.h

  exceptions/algorithm.h
  exceptions/base.h
  exceptions/config_block.h
  exceptions/config_block_io.h
  exceptions/io.h

  algo/algorithm.h
  algo/algorithm.txx
  algo/bundle_adjust.h
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
  camera_io.cxx
  config_block.cxx
  config_block_io.cxx
  feature.cxx
  image.cxx
  ins_data.cxx
  ins_data_io.cxx
  landmark.cxx
  landmark_map_io.cxx
  local_geo_cs.cxx
  matrix.cxx
  metrics.cxx
  rotation.cxx
  track.cxx
  track_set.cxx
  track_set_io.cxx
  vector.cxx

  exceptions/algorithm.cxx
  exceptions/base.cxx
  exceptions/config_block.cxx
  exceptions/config_block_io.cxx
  exceptions/io.cxx

  algo/algorithm.cxx
  algo/bundle_adjust.cxx
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
