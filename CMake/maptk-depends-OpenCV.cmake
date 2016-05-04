# Optionally find and configure OpenCV dependency

option( MAPTK_ENABLE_OPENCV
  "Enable OpenCV dependent code and plugins"
  OFF
  )

if( MAPTK_ENABLE_OPENCV )
  find_package( OpenCV REQUIRED )
  include_directories( SYSTEM ${OpenCV_INCLUDE_DIRS} )
  # Docs say we don't need to add link_directories() call for OpenCV lib dirs

  if( OpenCV_FOUND AND OpenCV_VERSION VERSION_LESS "2.4.6" )
    message(WARNING "Minimum support OpenCV version is 2.4.6, found ${OpenCV_VERSION}")
    set(OpenCV_FOUND FALSE)
  endif()
endif()
