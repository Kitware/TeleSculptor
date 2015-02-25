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

set(maptk_private_headers
  logging_macros.h
  )

set(maptk_sources
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
  registrar.cxx
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

maptk_install_headers(
  ${maptk_public_headers}
  )
maptk_add_library(
  maptk
  ${maptk_public_headers}
  ${maptk_private_headers}
  ${maptk_sources}
  )
target_link_libraries(maptk ${Boost_LIBRARIES})


###
#Cconfigure plugins
#
add_subdirectory(plugins)


###
# Build plugin management library
#
# This has to happen after plugin creation so as to be able to statically link
# in plugin libraries when building static.
#
option(MAPTK_USE_BUILD_PLUGIN_DIR
  "Whether to include the build-tree plugin directory to the default plugin path list"
  ON
  )

set(maptk_apm_headers
  algorithm_plugin_manager.h
  )
set(maptk_apm_headers_private
  algorithm_plugin_manager_static.h
  )
set(maptk_apm_sources
  algorithm_plugin_manager.cxx
  algorithm_plugin_manager_static.cxx
  )

maptk_install_headers(${maptk_apm_headers})
maptk_add_library(maptk_apm
  ${maptk_apm_headers}
  ${maptk_apm_headers_private}
  ${maptk_apm_sources}
  )
target_link_libraries(maptk_apm maptk)

# Configuring/Adding compile definitions to target
# (so we can use generator expressions)
if (WIN32)  # Windows puts DLLs in bin/CONFIG/
  set(shared_lib_dir "/bin")
else()  # Other Unix systems
  set(shared_lib_dir "/lib")
  set(lib_dir_suffix "${MAPTK_LIB_SUFFIX}")
endif()

if (CMAKE_CONFIGURATION_TYPES)
  set(config_subdir "/$<CONFIGURATION>")
endif()

# Convert use-build-dir switch into 1/0 value
macro(boolize var_name)
  if( ${var_name} )
    set(${var_name} 1)
  else()
    set(${var_name} 0)
  endif()
endmacro()

boolize(MAPTK_USE_BUILD_PLUGIN_DIR)
boolize(MAPTK_ENABLE_OPENCV)
boolize(MAPTK_ENABLE_PROJ)
boolize(MAPTK_ENABLE_VISCL)
boolize(MAPTK_ENABLE_VXL)

target_compile_definitions(maptk_apm
  PRIVATE
    # For plugin manager
    "SHARED_LIB_SUFFIX=\"${CMAKE_SHARED_MODULE_SUFFIX}\""
    "USE_BUILD_PLUGIN_DIR=${MAPTK_USE_BUILD_PLUGIN_DIR}"
    "DEFAULT_PLUGIN_DIR_BUILD=\"${MAPTK_BINARY_DIR}${shared_lib_dir}${config_subdir}/maptk\""
    "DEFAULT_PLUGIN_DIR_INSTALL=\"${CMAKE_INSTALL_PREFIX}${shared_lib_dir}${lib_dir_suffix}/maptk\""
    # For static interface
    "MAPTK_ENABLE_OPENCV=${MAPTK_ENABLE_OPENCV}"
    "MAPTK_ENABLE_PROJ=${MAPTK_ENABLE_PROJ}"
    "MAPTK_ENABLE_VISCL=${MAPTK_ENABLE_VISCL}"
    "MAPTK_ENABLE_VXL=${MAPTK_ENABLE_VXL}"
  )

if (NOT WIN32)
  target_link_libraries(maptk_apm ${CMAKE_DL_LIBS})
endif()

# When building static core, also link in static plugin libs
if(NOT BUILD_SHARED_LIBS)
  message(STATUS "Static build, linking plugin libraries into APM lib")
  get_property(plugin_static_libs
    GLOBAL PROPERTY maptk_plugin_libraries
    )
  message(STATUS "Static build \"plugin\" libraries: ${plugin_static_libs}")
  target_link_libraries(maptk_apm
    ${plugin_static_libs}
    )
endif()


###
# Marking source root for documentation generation
#
maptk_create_doxygen( maptk "${CMAKE_CURRENT_LIST_DIR}" )
