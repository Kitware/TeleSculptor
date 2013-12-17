project(maptk_test_core)

set(test_libraries
  maptk_core)

##############################
# Core tests
##############################
maptk_discover_tests(core_vector test_libraries test_vector.cxx)
maptk_discover_tests(core_rotation test_libraries test_rotation.cxx)
maptk_discover_tests(core_image test_libraries test_image.cxx)
maptk_discover_tests(core_ins_data test_libraries test_ins_data.cxx)
maptk_discover_tests(core_config test_libraries test_config.cxx)
maptk_discover_tests(core_config_io test_libraries test_config_io.cxx)
