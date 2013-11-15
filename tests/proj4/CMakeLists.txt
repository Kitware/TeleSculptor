project(maptk_test_proj4)

set(test_libraries
  maptk_core
  maptk_proj4
  ${PROJ4_LIBRARY}
  )

#####################
# MAPTK Proj4 tests #
#####################
maptk_discover_tests(proj4_geo_map test_libraries test_geo_map.cxx)
