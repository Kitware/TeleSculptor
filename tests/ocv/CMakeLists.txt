project(maptk_test_ocv)

set(test_libraries
  maptk_ocv)

##############################
# MAPTK OpenCV tests
##############################
maptk_discover_tests(ocv_estimate_homography test_libraries test_estimate_homography.cxx)
maptk_discover_tests(ocv_image               test_libraries test_image.cxx)
maptk_discover_tests(ocv_feature_set         test_libraries test_feature_set.cxx)
maptk_discover_tests(ocv_descriptor_set      test_libraries test_descriptor_set.cxx)
maptk_discover_tests(ocv_match_set           test_libraries test_match_set.cxx)
maptk_discover_tests(ocv_algo_params         test_libraries test_algo_param_extraction.cxx)
