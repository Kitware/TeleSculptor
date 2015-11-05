project(maptk_test_vxl)

set(test_libraries maptk_vxl)

##############################
# MAPTK VXL tests
##############################
kwiver_discover_tests(vxl_bundle_adjust                test_libraries test_bundle_adjust.cxx)
kwiver_discover_tests(vxl_camera                       test_libraries test_camera.cxx)
kwiver_discover_tests(vxl_estimate_essential_matrix    test_libraries test_estimate_essential_matrix.cxx)
kwiver_discover_tests(vxl_estimate_homography          test_libraries test_estimate_homography.cxx)
kwiver_discover_tests(vxl_estimate_similarity          test_libraries test_estimate_similarity.cxx)
kwiver_discover_tests(vxl_image                        test_libraries test_image.cxx)
kwiver_discover_tests(vxl_optimize_cameras             test_libraries test_optimize_cameras.cxx)
kwiver_discover_tests(vxl_triangulate_landmarks        test_libraries test_triangulate_landmarks.cxx)

# This test is really for an algorithm in core, but it's here
# because the test depends on availablity of VXL sub-algorithms
set(test_libraries maptk_vxl maptk_core)
kwiver_discover_tests(vxl_initialize_cameras_landmarks test_libraries test_initialize_cameras_landmarks.cxx)
