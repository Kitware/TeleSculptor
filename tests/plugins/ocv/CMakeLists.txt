project(maptk_test_ocv)

set(test_libraries
  maptk_ocv )

##############################
# MAPTK OpenCV tests
##############################
kwiver_discover_tests(ocv_estimate_homography test_libraries test_estimate_homography.cxx)
kwiver_discover_tests(ocv_image               test_libraries test_image.cxx)
kwiver_discover_tests(ocv_feature_set         test_libraries test_feature_set.cxx)
kwiver_discover_tests(ocv_descriptor_set      test_libraries test_descriptor_set.cxx)
kwiver_discover_tests(ocv_match_set           test_libraries test_match_set.cxx)
kwiver_discover_tests(ocv_algo_params         test_libraries test_algo_param_extraction.cxx)
kwiver_discover_tests(ocv_test_distortion     test_libraries test_distortion.cxx)
