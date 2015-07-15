project(maptk_test_core)

set(test_libraries maptk vital vital_apm)

##############################
# Core tests
##############################
maptk_discover_tests(core_algo               test_libraries test_algo.cxx)

maptk_discover_tests(core_est                test_libraries test_estimate_similarity_transform.cxx)
