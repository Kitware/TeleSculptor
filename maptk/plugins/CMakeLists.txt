add_subdirectory( default )

# Add OpenCV plugin if enabled
cmake_dependent_option( MAPTK_PLUGIN_ENABLE_OPENCV
  "Enable OpenCV plug-in building / installation" OFF
  "MAPTK_ENABLE_OPENCV" OFF
  )
if( MAPTK_PLUGIN_ENABLE_OPENCV )
  add_subdirectory( ocv )
endif()

# Add VXL plugin if enabled
cmake_dependent_option( MAPTK_PLUGIN_ENABLE_VXL
  "Enable VXL plugin building / installation" OFF
  "MAPTK_ENABLE_VXL" OFF
  )
if( MAPTK_PLUGIN_ENABLE_VXL )
  add_subdirectory( vxl )
endif()
