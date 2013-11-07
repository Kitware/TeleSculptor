project(maptk_test_core)

set(test_libraries
  maptk_core)

##############################
# Core tests
##############################
maptk_discover_tests(core_vector test_libraries test_vector.cxx)
maptk_discover_tests(core_rotation test_libraries test_rotation.cxx)
maptk_discover_tests(core_image test_libraries test_image.cxx)

