project(maptk_test_core)

set(test_libraries maptk maptk_apm)

##############################
# Core tests
##############################
maptk_discover_tests(core_algo               test_libraries test_algo.cxx)
maptk_discover_tests(core_camera             test_libraries test_camera.cxx)
maptk_discover_tests(core_camera_io          test_libraries test_camera_io.cxx
  "${maptk_test_data_directory}"
  )
maptk_discover_tests(core_camera_intrinsics  test_libraries test_camera_intrinsics.cxx)
maptk_discover_tests(core_config             test_libraries test_config_block.cxx)
maptk_discover_tests(core_config_io          test_libraries test_config_block_io.cxx
  "${maptk_test_data_directory}"
  )
maptk_discover_tests(core_est                test_libraries test_estimate_similarity_transform.cxx)
maptk_discover_tests(core_homography         test_libraries test_homography.cxx)
maptk_discover_tests(core_image              test_libraries test_image.cxx)
maptk_discover_tests(core_ins_data           test_libraries test_ins_data.cxx)
maptk_discover_tests(core_rotation           test_libraries test_rotation.cxx)
maptk_discover_tests(core_similarity         test_libraries test_similarity.cxx)
maptk_discover_tests(core_track              test_libraries test_track.cxx)
maptk_discover_tests(core_track_set          test_libraries test_track_set.cxx)
maptk_discover_tests(core_transform          test_libraries test_transform.cxx)
maptk_discover_tests(core_vector             test_libraries test_vector.cxx)
