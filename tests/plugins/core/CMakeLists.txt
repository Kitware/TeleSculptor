project(maptk_test_core)

set( test_libraries    maptk_core vital )

##############################
# MAPTK core plugin tests
##############################
kwiver_discover_tests(core_estimate_canonical_transform test_libraries test_estimate_canonical_transform.cxx)
kwiver_discover_tests(core_triangulate_landmarks        test_libraries test_triangulate_landmarks.cxx)
