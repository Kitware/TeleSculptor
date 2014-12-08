# Optionally find and configure OpenCV dependency

option( MAPTK_ENABLE_OPENCV
  "Enable OpenCV dependent code and plugins"
  OFF
  )

if( MAPTK_ENABLE_OPENCV )
  find_package( OpenCV REQUIRED )
  include_directories( SYSTEM ${OpenCV_INCLUDE_DIRS} )
  # Docs say we don't need to add link_directories() call for OpenCV lib dirs

  # NOTE: Artifact carried over from before re-design. What is this for? Ask Matt L.
  set(USE_OPENCV_VERSION MAPTK_ENABLE_PLUGIN_OPENCV AND OpenCV_FOUND AND OpenCV_VERSION VERSION_GREATER "2.4")
endif()
