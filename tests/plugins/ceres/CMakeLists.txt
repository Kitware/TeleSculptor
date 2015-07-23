project(maptk_test_ceres)

set(test_libraries maptk_ceres vital)

##############################
# MAPTK Ceres tests
##############################
maptk_discover_tests(ceres_bundle_adjust       test_libraries test_bundle_adjust.cxx)
maptk_discover_tests(ceres_reprojection_error  test_libraries test_reprojection_error.cxx)
