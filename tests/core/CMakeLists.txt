project(maptk_test_core)

set(test_libraries
  maptk_core)

##############################
# Core tests
##############################
maptk_discover_tests(point_2d test_libraries test_point_2d.cxx)

