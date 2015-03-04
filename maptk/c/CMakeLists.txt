# C interface library
include(CMakeDependentOption)

option(MAPTK_ENABLE_C_LIB
  "Enable building of C interface library."
  OFF
  )
cmake_dependent_option( MAPTK_C_LIB_FORCE_SHARED
  "Force C interface library to be build shared."
  OFF  # Default value
  "MAPTK_ENABLE_C_LIB; NOT BUILD_SHARED_LIBS"
  OFF # State when dependency expression is false
  )


if( MAPTK_ENABLE_C_LIB )

  set(maptk_c_public_headers
    config.h

    algorithm.h
    algorithm_plugin_manager.h
    camera.h
    camera_map.h
    common.h
    config_block.h
    error_handle.h
    image.h
    image_container.h
    track.h
    track_set.h

    algo_def/convert_image.h
    algo_def/image_io.h
    algo_def/track_features.h
    )
  set(maptk_c_private_headers
    helpers/algorithm.h
    helpers/c_utils.h
    helpers/camera.h
    helpers/camera_map.h
    helpers/config_block.h
    helpers/image_container.h
    helpers/track.h
    helpers/track_set.h
    )
  set(maptk_c_sources
    algorithm.cxx
    algorithm_plugin_manager.cxx
    camera.cxx
    camera_map.cxx
    common.cxx
    config_block.cxx
    error_handle.cxx
    image.cxx
    image_container.cxx
    track.cxx
    track_set.cxx

    algo_def/convert_image.cxx
    algo_def/image_io.cxx
    algo_def/track_features.cxx

    helpers/c_utils.cxx
    )

  if( MAPTK_C_LIB_FORCE_SHARED )
    set(maptk_force_shared_lib SHARED)
  endif()

  maptk_install_headers(
    ${maptk_c_public_headers}
    SUBDIR c
    )
  maptk_add_library( maptk_c
    ${maptk_force_shared_lib}
    ${maptk_c_public_headers}
    ${maptk_c_private_headers}
    ${maptk_c_sources}
    )
  target_link_libraries( maptk_c
    maptk maptk_apm
    )

endif( MAPTK_ENABLE_C_LIB )
