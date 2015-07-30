project(maptk_test_maptk)

set(test_libraries maptk maptk_apm)

##############################
# MAP-Tk library tests
##############################
maptk_discover_tests(maptk_algo               test_libraries test_algo.cxx)
maptk_discover_tests(maptk_camera             test_libraries test_camera.cxx)
maptk_discover_tests(maptk_camera_io          test_libraries test_camera_io.cxx
  "${maptk_test_data_directory}"
  )
maptk_discover_tests(maptk_camera_intrinsics  test_libraries test_camera_intrinsics.cxx)
maptk_discover_tests(maptk_config             test_libraries test_config_block.cxx)
maptk_discover_tests(maptk_config_io          test_libraries test_config_block_io.cxx
  "${maptk_test_data_directory}"
  )
maptk_discover_tests(maptk_est                test_libraries test_estimate_similarity_transform.cxx)
maptk_discover_tests(maptk_homography         test_libraries test_homography.cxx)
maptk_discover_tests(maptk_image              test_libraries test_image.cxx)
maptk_discover_tests(maptk_ins_data           test_libraries test_ins_data.cxx)
maptk_discover_tests(maptk_rotation           test_libraries test_rotation.cxx)
maptk_discover_tests(maptk_similarity         test_libraries test_similarity.cxx)
maptk_discover_tests(maptk_track              test_libraries test_track.cxx)
maptk_discover_tests(maptk_track_set          test_libraries test_track_set.cxx)
maptk_discover_tests(maptk_transform          test_libraries test_transform.cxx)
maptk_discover_tests(maptk_vector             test_libraries test_vector.cxx)
