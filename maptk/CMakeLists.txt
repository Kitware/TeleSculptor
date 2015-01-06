include(CheckCXXCompilerFlag)
check_cxx_compiler_flag(-fvisibility=hidden MAPTK_HAVE_GCC_VISIBILITY)

# Configure general config.h.in
maptk_configure_file(config.h
  "${CMAKE_CURRENT_SOURCE_DIR}/config.h.in"
  "${CMAKE_CURRENT_BINARY_DIR}/config.h"
  BUILD_SHARED_LIBS
  MAPTK_HAVE_GCC_VISIBILITY
  )
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/config.h"
  DESTINATION include/maptk
  )


include_directories("${MAPTK_SOURCE_DIR}")
include_directories("${MAPTK_BINARY_DIR}")


set(maptk_public_headers
  algorithm_plugin_manager.h
  camera.h
  camera_intrinsics.h
  camera_io.h
  camera_map.h
  camera_map_io.h
  config_block.h
  config_block_io.h
  covariance.h
  descriptor.h
  descriptor_set.h
  eigen_io.h
  exceptions.h
  feature.h
  feature_set.h
  geo_reference_points_io.h
  homography.h
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
  projected_track_set.h
  rotation.h
  registrar.h
  similarity.h
  track.h
  track_set.h
  track_set_io.h
  transform.h
  types.h
  vector.h

  exceptions/algorithm.h
  exceptions/base.h
  exceptions/config_block.h
  exceptions/config_block_io.h
  exceptions/image.h
  exceptions/io.h
  exceptions/math.h

  algo/algorithm.h
  algo/algorithm.txx
  algo/analyze_tracks.h
  algo/bundle_adjust.h
  algo/close_loops.h
  algo/compute_ref_homography.h
  algo/convert_image.h
  algo/detect_features.h
  algo/draw_tracks.h
  algo/estimate_essential_matrix.h
  algo/estimate_homography.h
  algo/estimate_similarity_transform.h
  algo/extract_descriptors.h
  algo/geo_map.h
  algo/image_io.h
  algo/initialize_cameras_landmarks.h
  algo/match_features.h
  algo/optimize_cameras.h
  algo/track_features.h
  algo/triangulate_landmarks.h

  plugin_interface/algorithm_plugin_interface.h
  plugin_interface/algorithm_plugin_interface_macros.h
  )

set(maptk_sources
  algorithm_plugin_manager.cxx
  camera.cxx
  camera_intrinsics.cxx
  camera_io.cxx
  camera_map_io.cxx
  config_block.cxx
  config_block_io.cxx
  feature.cxx
  geo_reference_points_io.cxx
  homography.cxx
  image.cxx
  ins_data.cxx
  ins_data_io.cxx
  landmark.cxx
  landmark_map_io.cxx
  local_geo_cs.cxx
  metrics.cxx
  projected_track_set.cxx
  rotation.cxx
  similarity.cxx
  track.cxx
  track_set.cxx
  track_set_io.cxx
  transform.cxx

  exceptions/algorithm.cxx
  exceptions/base.cxx
  exceptions/config_block.cxx
  exceptions/config_block_io.cxx
  exceptions/image.cxx
  exceptions/io.cxx
  exceptions/math.cxx

  algo/algorithm.cxx
  algo/analyze_tracks.cxx
  algo/bundle_adjust.cxx
  algo/close_loops.cxx
  algo/compute_ref_homography.cxx
  algo/convert_image.cxx
  algo/detect_features.cxx
  algo/draw_tracks.cxx
  algo/estimate_essential_matrix.cxx
  algo/estimate_homography.cxx
  algo/estimate_similarity_transform.cxx
  algo/extract_descriptors.cxx
  algo/geo_map.cxx
  algo/image_io.cxx
  algo/initialize_cameras_landmarks.cxx
  algo/match_features.cxx
  algo/optimize_cameras.cxx
  algo/track_features.cxx
  algo/triangulate_landmarks.cxx
  )


# Setting up configured properties on plugin manager source files
option(MAPTK_USE_BUILD_PLUGIN_DIR
  "Whether to include the build-tree plugin directory to the default plugin search path list"
  ON
  )
set(plugin_manager_sources
  algorithm_plugin_manager.h  algorithm_plugin_manager.cxx
  )
if (WIN32) # Windows puts DLLs in the bin directory
  set(rel_plugin_dir "bin")
else()
  set(rel_plugin_dir "lib")
  set(plugin_lib_suffix "${MAPTK_LIB_SUFFIX}")
endif() # TODO: Apple Xcode specific subdir?
# LIBRARY_SUFFIX here refers to the filename suffix, e.g. .so / .dylib / .dll / etc.
set(plugin_src_definitions
  "LIBRARY_SUFFIX=\"${CMAKE_SHARED_MODULE_SUFFIX}\""
  "DEFAULT_PLUGIN_DIR_BUILD=\"${MAPTK_BINARY_DIR}/${rel_plugin_dir}/maptk\""
  "DEFAULT_PLUGIN_DIR_INSTALL=\"${CMAKE_INSTALL_PREFIX}/${rel_plugin_dir}${plugin_lib_suffix}/maptk\""
  "USE_BUILD_PLUGIN_DIR=${MAPTK_USE_BUILD_PLUGIN_DIR}"
  )
set_source_files_properties(
  ${plugin_manager_sources}
  PROPERTIES
    COMPILE_DEFINITIONS "${plugin_src_definitions}"
  )


maptk_install_headers(
  ${maptk_public_headers}
  )
maptk_add_library(
  maptk
  ${maptk_public_headers}
  ${maptk_sources}
  )
target_link_libraries(maptk ${Boost_LIBRARIES})

# For plug-in management
if (NOT WIN32)
  target_link_libraries(maptk dl)
endif()

# configure plugins
add_subdirectory(plugins)

maptk_create_doxygen( maptk "${CMAKE_CURRENT_LIST_DIR}" )
