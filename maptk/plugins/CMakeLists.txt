# Base plug-in with default implementations
add_subdirectory( core )

# Add Ceres plugin if enabled
if( MAPTK_ENABLE_CERES )
  add_subdirectory( ceres )
endif()

# Add OpenCV plugin if enabled
if( MAPTK_ENABLE_OPENCV )
  add_subdirectory( ocv )
endif()

# Add PROJ plugin if enabled
if( MAPTK_ENABLE_PROJ )
  add_subdirectory( proj )
endif()

#Add VisCL plugin if enabled
if( MAPTK_ENABLE_VISCL )
  add_subdirectory( viscl )
endif()

# Add VXL plugin if enabled
if( MAPTK_ENABLE_VXL )
  add_subdirectory( vxl )
endif()
