# Optional find and confgure VXL dependency

option( MAPTK_ENABLE_VXL
  "Enable VXL dependent code and plugins"
  OFF
  )

if( MAPTK_ENABLE_VXL )
  find_package( VXL REQUIRED )
  include_directories( SYSTEM ${VXL_CORE_INCLUDE_DIR} )
  include_directories( SYSTEM ${VXL_VCL_INCLUDE_DIR} )
  include_directories( SYSTEM ${VXL_RPL_INCLUDE_DIR} )
  link_directories( ${VXL_LIBRARY_DIR} )
endif( MAPTK_ENABLE_VXL )
