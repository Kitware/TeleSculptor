# Tools

include_directories("${MAPTK_SOURCE_DIR}")
include_directories("${MAPTK_BINARY_DIR}")

get_property(maptk_libs GLOBAL PROPERTY maptk_libraries)

maptk_add_executable(maptk_track_features track_features.cxx)
target_link_libraries(maptk_track_features
  LINK_PRIVATE
    ${maptk_libs})
