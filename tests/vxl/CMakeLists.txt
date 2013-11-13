project(maptk_test_vxl)

set(test_libraries
  maptk_vxl)

##############################
# MAPTK VXL tests
##############################
maptk_discover_tests(vxl_image test_libraries test_image.cxx)
