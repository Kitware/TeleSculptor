project(maptk_test_ceres)

set(test_libraries maptk_ceres)

##############################
# MAPTK Ceres tests
##############################
kwiver_discover_tests(ceres_bundle_adjust       test_libraries test_bundle_adjust.cxx)
kwiver_discover_tests(ceres_reprojection_error  test_libraries test_reprojection_error.cxx)
