# Base plug-in with default implementations
add_subdirectory( default )

# Add OpenCV plugin if enabled
cmake_dependent_option( MAPTK_PLUGIN_ENABLE_OPENCV
  "Enable OpenCV plug-in building / installation" OFF
  "MAPTK_ENABLE_OPENCV;OpenCV_FOUND" OFF
  )
if( MAPTK_PLUGIN_ENABLE_OPENCV )
  add_subdirectory( ocv )
endif()

# Add PROJ plugin if enabled
cmake_dependent_option( MAPTK_PLUGIN_ENABLE_PROJ
  "Enable PROJ plug-in building / installation" OFF
  "MAPTK_ENABLE_PROJ;PROJ_FOUND" OFF
  )
if( MAPTK_PLUGIN_ENABLE_PROJ )
  add_subdirectory( proj )
endif()

#Add VisCL plugin if enabled
cmake_dependent_option( MAPTK_PLUGIN_ENABLE_VISCL
  "Enable VisCL plug-in building / installation" OFF
  "MAPTK_ENABLE_VISCL;viscl_FOUND" OFF
  )
if( MAPTK_PLUGIN_ENABLE_VISCL )
  add_subdirectory( viscl )
endif()

# Add VXL plugin if enabled
cmake_dependent_option( MAPTK_PLUGIN_ENABLE_VXL
  "Enable VXL plugin building / installation" OFF
  "MAPTK_ENABLE_VXL;VXL_FOUND" OFF
  )
if( MAPTK_PLUGIN_ENABLE_VXL )
  add_subdirectory( vxl )
endif()
