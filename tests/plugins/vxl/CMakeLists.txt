project(maptk_test_vxl)

set(test_libraries maptk_vxl)

##############################
# MAPTK VXL tests
##############################
maptk_discover_tests(vxl_bundle_adjust                test_libraries test_bundle_adjust.cxx)
maptk_discover_tests(vxl_camera                       test_libraries test_camera.cxx)
maptk_discover_tests(vxl_estimate_essential_matrix    test_libraries test_estimate_essential_matrix.cxx)
maptk_discover_tests(vxl_estimate_homography          test_libraries test_estimate_homography.cxx)
maptk_discover_tests(vxl_estimate_similarity          test_libraries test_estimate_similarity.cxx)
maptk_discover_tests(vxl_image                        test_libraries test_image.cxx)
maptk_discover_tests(vxl_initialize_cameras_landmarks test_libraries test_initialize_cameras_landmarks.cxx)
maptk_discover_tests(vxl_optimize_cameras             test_libraries test_optimize_cameras.cxx)
maptk_discover_tests(vxl_triangulate_landmarks        test_libraries test_triangulate_landmarks.cxx)
