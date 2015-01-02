project(maptk_test_proj)

set(test_libraries
  maptk_proj
  )

#####################
# MAPTK Proj tests #
#####################
maptk_discover_tests(proj_geo_map test_libraries test_geo_map.cxx)
