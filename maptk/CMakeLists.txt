project(MAPTK_MODULES)

add_subdirectory(core)

set(MAPTK_LIBRARIES
  "maptk_core"
  CACHE INTERNAL ""
  )

set(DOCUMENTATION_OUTPUT_PATH
  ${CMAKE_BINARY_DIR}/doc
  )
maptk_create_doxygen(
  ${DOCUMENTATION_OUTPUT_PATH}
  ${CMAKE_CURRENT_SOURCE_DIR}/core
  core
  )
