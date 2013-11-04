# maptk/vxl

set( maptk_vxl_sources
  image_container.h        image_container.cxx
  vil_image_memory.h       vil_image_memory.cxx
#  image_io.h               image_io.cxx
)



add_library( maptk_vxl ${maptk_vxl_sources} )
set_target_properties( maptk_vxl PROPERTIES
  VERSION ${maptk_VERSION} SOVERSION ${maptk_VERSION}
)

target_link_libraries( maptk_vxl maptk_core vil )

install( TARGETS maptk_vxl EXPORT maptk
  RUNTIME DESTINATION bin LIBRARY DESTINATION lib ARCHIVE DESTINATION lib
)
install( DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} DESTINATION include/maptk
  FILES_MATCHING PATTERN "*.h"
)
export( TARGETS maptk_vxl APPEND FILE "${MAPTK_EXPORT_FILE}")
