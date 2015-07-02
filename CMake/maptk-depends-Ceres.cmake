# Optionally find and configure Ceres dependency

option( MAPTK_ENABLE_CERES
  "Enable Ceres dependent code and plugins"
  OFF
  )

if( MAPTK_ENABLE_CERES )
  find_package( Ceres 1.10.0 REQUIRED )
  include_directories( SYSTEM ${CERES_INCLUDE_DIRS} )
endif()
