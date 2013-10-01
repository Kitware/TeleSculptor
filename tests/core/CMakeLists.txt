project(maptk_test_core)

set(test_libraries
  maptk_core)

##############################
# Core tests
##############################
maptk_discover_tests(vector test_libraries test_vector.cxx)

