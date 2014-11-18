# maptk/ocv

option(MAPTK_ENABLE_OPENCV "Enable OpenCV-dependent code" OFF)
if(MAPTK_ENABLE_OPENCV)
  find_package(OpenCV REQUIRED)
  include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS})

  set(maptk_ocv_headers_public
    analyze_tracks.h
    descriptor_set.h
    detect_features.h
    draw_tracks.h
    estimate_homography.h
    extract_descriptors.h
    feature_set.h
    image_container.h
    image_io.h
    mat_image_memory.h
    match_features.h
    match_set.h
    matrix.h
    ocv_algo_tools.h
    ocv_config.h
    )

  set(maptk_ocv_sources
    analyze_tracks.cxx
    descriptor_set.cxx
    detect_features.cxx
    draw_tracks.cxx
    estimate_homography.cxx
    extract_descriptors.cxx
    feature_set.cxx
    image_container.cxx
    image_io.cxx
    mat_image_memory.cxx
    match_features.cxx
    match_set.cxx
    ocv_algo_tools.cxx

    algorithm_impl_pl_interface.cxx
    )

  maptk_add_plugin(ocv MAKE_PLUGIN_OCV
    ${maptk_ocv_headers_public}
    ${maptk_ocv_sources}
    )
  target_link_libraries(ocv
    maptk
    ${OpenCV_LIBS}
    )


endif(MAPTK_ENABLE_OPENCV)
