project(maptk_test_vxl)

set(test_libraries
  maptk_vxl)

##############################
# MAPTK VXL tests
##############################
maptk_discover_tests(vxl_bundle_adjust         test_libraries test_bundle_adjust.cxx)
maptk_discover_tests(vxl_camera                test_libraries test_camera.cxx)
maptk_discover_tests(vxl_estimate_homography   test_libraries test_estimate_homography.cxx)
maptk_discover_tests(vxl_estimate_similarity   test_libraries test_estimate_similarity.cxx)
maptk_discover_tests(vxl_image                 test_libraries test_image.cxx)
maptk_discover_tests(vxl_triangulate_landmarks test_libraries test_triangulate_landmarks.cxx)
