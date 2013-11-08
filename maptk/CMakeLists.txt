project(MAPTK_MODULES)

message(STATUS "Entering core module")
add_subdirectory(core)

set(MAPTK_LIBRARIES
  "maptk_core"
  CACHE INTERNAL ""
  )

message(STATUS "Calling doxygen create on core")
maptk_create_doxygen(core
  ${CMAKE_CURRENT_SOURCE_DIR}/core
  )
