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


###
# Setting up core library
#
set(maptk_public_headers
  metrics.h

  local_geo_cs.h
  projected_track_set.h
  geo_reference_points_io.h
  ins_data.h
  ins_data_io.h

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
  algo/filter_features.h
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
  metrics.cxx

  local_geo_cs.cxx
  projected_track_set.cxx
  geo_reference_points_io.cxx
  ins_data.cxx
  ins_data_io.cxx

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
  algo/filter_features.cxx
  algo/geo_map.cxx
  algo/image_io.cxx
  algo/initialize_cameras_landmarks.cxx
  algo/match_features.cxx
  algo/optimize_cameras.cxx
  algo/track_features.cxx
  algo/triangulate_landmarks.cxx
  )

maptk_install_headers(
  ${maptk_public_headers}
  )
maptk_add_library(
  maptk
  ${maptk_public_headers}
  ${maptk_sources}
  )
target_link_libraries(maptk ${Boost_LIBRARIES} ${KWIVER_LIBRARIES} )


###
#Cconfigure plugins
#
add_subdirectory(plugins)


# Configuring/Adding compile definitions to target
# (so we can use generator expressions)

if (CMAKE_CONFIGURATION_TYPES)
  set(config_subdir "/$<CONFIGURATION>")
endif()


###
# C Interface
#
### add_subdirectory(c)
### add_subdirectory(python)


###
# Marking source root for documentation generation
#
maptk_create_doxygen( maptk "${CMAKE_CURRENT_LIST_DIR}" )
