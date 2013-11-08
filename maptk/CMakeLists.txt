project(MAPTK_MODULES)

message(STATUS "Entering core module")
maptk_add_module(core "${CMAKE_CURRENT_LIST_DIR}/core")


set(MAPTK_LIBRARIES
  "maptk_core"
  CACHE INTERNAL ""
  )
