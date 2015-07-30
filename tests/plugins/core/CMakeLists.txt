project(maptk_test_core)

set(test_libraries maptk_core)

##############################
# MAPTK core plugin tests
##############################
maptk_discover_tests(core_triangulate_landmarks        test_libraries test_triangulate_landmarks.cxx)
