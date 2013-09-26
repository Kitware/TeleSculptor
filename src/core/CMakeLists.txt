# maptk/core

set( maptk_core_sources
  camera.h
  camera_intrinsics.h
  covariance_2d.h
  covariance_3d.h
  descriptor.h
  feature.h                feature.cxx
  landmark.h
  point_2d.h
  point_3d.h
)



add_library( maptk_core ${maptk_core_sources} )
set_target_properties( maptk_core PROPERTIES
  VERSION ${maptk_VERSION} SOVERSION ${maptk_VERSION}
)

#target_link_libraries( maptk_core )

install( TARGETS maptk_core EXPORT maptk
  RUNTIME DESTINATION bin LIBRARY DESTINATION lib ARCHIVE DESTINATION lib
)
install( DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} DESTINATION include
  FILES_MATCHING PATTERN "*.h"
)
export( TARGETS maptk_core APPEND FILE "${MAPTK_EXPORT_FILE}")
