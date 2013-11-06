project(MAPTK_MODULES)

add_subdirectory(core)

set(MAPTK_LIBRARIES
  "maptk_core"
  CACHE INTERNAL ""
  )

maptk_create_doxygen(
  ${MAPTK_BUILD_DEST_DOC}
  ${CMAKE_CURRENT_SOURCE_DIR}/core
  core
  )
