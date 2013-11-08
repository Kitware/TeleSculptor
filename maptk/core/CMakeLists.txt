# maptk/core

set( maptk_core_sources
  camera.h                 camera.cxx
  camera_intrinsics.h      camera_intrinsics.cxx
  covariance.h
  descriptor.h
  descriptor_set.h
  feature.h                feature.cxx
  feature_set.h
  image.h                  image.cxx
  image_container.h
  landmark.h
  matrix.h                 matrix.cxx
  rotation.h               rotation.cxx
  track.h                  track.cxx
  vector.h                 vector.cxx
  vector_cmath.h

  algo/image_io.h
  algo/detect_features.h
  algo/extract_descriptors.h
  )

# installing header files
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  DESTINATION include/maptk
  FILES_MATCHING PATTERN "*.h"
  )

add_library(maptk_core ${maptk_core_sources})
set_target_properties(maptk_core PROPERTIES
  VERSION ${MAPTK_VERSION}
  SOVERSION ${MAPTK_VERSION}
  ARCHIVE_OUTPUT_DIRECTORY "${MAPTK_BINARY_DIR}/lib"
  LIBRARY_OUTPUT_DIRECTORY "${MAPTK_BINARY_DIR}/lib"
  RUNTIME_OUTPUT_DIRECTORY "${MAPTK_BINARY_DIR}/bin"
  )
install(TARGETS maptk_core
  EXPORT maptk
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  )
export(TARGETS maptk_core APPEND FILE "${MAPTK_EXPORT_FILE}")
