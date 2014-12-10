# Optionally find and configure VisCL dependency

option( MAPTK_ENABLE_VISCL
  "Enable VidCL dependent code and plugins"
  OFF
  )

if( MAPTK_ENABLE_VISCL )
  find_package( viscl REQUIRED )
  include_directories( SYSTEM ${viscl_INCLUDE_DIR} )
  include_directories( SYSTEM ${viscl_OPENCL_INCLUDE_DIRS} )
endif( MAPTK_ENABLE_VISCL )
