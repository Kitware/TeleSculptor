project(maptk_test_maptk)

set(test_libraries maptk)

##############################
# MAP-Tk library tests
##############################
kwiver_discover_tests(maptk_camera             test_libraries test_camera.cxx)
