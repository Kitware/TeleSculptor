# Add sets of tests based on which plug-ins are enabled

if ( MAPTK_ENABLE_OPENCV AND MAPTK_PLUGIN_ENABLE_OPENCV )
  add_subdirectory(ocv)
endif()

if( MAPTK_ENABLE_PROJ AND MAPTK_PLUGIN_ENABLE_PROJ )
  add_subdirectory(proj)
endif()

if ( MAPTK_ENABLE_VXL AND MAPTK_PLUGIN_ENABLE_VXL )
  add_subdirectory(vxl)
endif()
