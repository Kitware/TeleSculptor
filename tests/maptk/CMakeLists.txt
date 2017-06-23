project(maptk_test_maptk)

set(test_libraries maptk kwiver::kwiver_algo_core)

##############################
# MAP-Tk library tests
##############################
kwiver_discover_tests(maptk_epipolar_geometry    test_libraries test_epipolar_geometry.cxx)
kwiver_discover_tests(maptk_interpolate_camera   test_libraries test_interpolate_camera.cxx)
