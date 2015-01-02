# Optionally find and configure PROJ dependency

option( MAPTK_ENABLE_PROJ
  "Enable PROJ dependent code and plugins"
  OFF
  )

if( MAPTK_ENABLE_PROJ )
  find_package( PROJ REQUIRED )
  include_directories( SYSTEM ${PROJ_INCLUDE_DIR} )
endif( MAPTK_ENABLE_PROJ )
