project(maptk_test_ocv)

set(test_libraries
  maptk_ocv)

##############################
# MAPTK OpenCV tests
##############################
maptk_discover_tests(ocv_image test_libraries test_image.cxx)
