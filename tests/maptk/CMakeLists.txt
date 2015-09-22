project(maptk_test_maptk)

set(test_libraries maptk)

##############################
# MAP-Tk library tests
##############################
kwiver_discover_tests(maptk_interpolate_camera   test_libraries test_interpolate_camera.cxx)
