# Tools

include_directories("${TELESCULPTOR_SOURCE_DIR}")
include_directories("${TELESCULPTOR_BINARY_DIR}")

get_property(telesculptor_libs GLOBAL PROPERTY telesculptor_libraries)

kwiver_add_executable(maptk_detect_and_describe detect_and_describe.cxx)
target_link_libraries(maptk_detect_and_describe
  PRIVATE             maptk
                      kwiver::kwiver_algo_core
                      kwiver::vital_algo
                      kwiver::vital_vpm
                      kwiver::kwiversys
    )

kwiver_add_executable(maptk_track_features track_features.cxx)
target_link_libraries(maptk_track_features
  PRIVATE             maptk
                      kwiver::kwiver_algo_core
                      kwiver::vital_algo
                      kwiver::vital_vpm
                      kwiver::kwiversys
    )

kwiver_add_executable(maptk_match_matrix match_matrix.cxx)
target_link_libraries(maptk_match_matrix
  PRIVATE             maptk
                      kwiver::kwiver_algo_core
                      kwiver::kwiversys
  )

kwiver_add_executable(maptk_pos2krtd pos2krtd.cxx)
target_link_libraries(maptk_pos2krtd
  PRIVATE             maptk
                      kwiver::vital_algo
                      kwiver::vital_vpm
  )

kwiver_add_executable(maptk_bundle_adjust_tracks bundle_adjust_tracks.cxx)
target_link_libraries(maptk_bundle_adjust_tracks
  PRIVATE             maptk
                      kwiver::kwiver_algo_core
                      kwiver::kwiver_algo_mvg
                      kwiver::vital_vpm
                      kwiver::kwiversys
  )

kwiver_add_executable(maptk_apply_gcp apply_gcp.cxx)
target_link_libraries(maptk_apply_gcp
  PRIVATE             maptk
                      kwiver::kwiver_algo_mvg
                      kwiver::vital_vpm
                      kwiver::kwiversys
  )

kwiver_add_executable(maptk_analyze_tracks analyze_tracks.cxx)
target_link_libraries(maptk_analyze_tracks
  PRIVATE             maptk
                      kwiver::kwiver_algo_core
                      kwiver::kwiver_algo_mvg
                      kwiver::vital_vpm
                      kwiver::kwiversys
  )

kwiver_add_executable(maptk_estimate_homography estimate_homography.cxx)
target_link_libraries(maptk_estimate_homography
  PRIVATE             maptk
                      kwiver::vital_algo
                      kwiver::vital_vpm
                      kwiver::kwiversys
  )
