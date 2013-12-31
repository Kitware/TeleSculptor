# Tools

include_directories("${MAPTK_SOURCE_DIR}")
include_directories("${MAPTK_BINARY_DIR}")

get_property(maptk_libs GLOBAL PROPERTY maptk_libraries)

maptk_add_executable(maptk_track_features track_features.cxx)
target_link_libraries(maptk_track_features
  LINK_PRIVATE
    ${maptk_libs})

maptk_add_executable(maptk_pos2krtd pos2krtd.cxx)
target_link_libraries(maptk_pos2krtd
  LINK_PRIVATE
    ${maptk_libs}
    ${Boost_SYSTEM_LIBRARY}
    ${Boost_FILESYSTEM_LIBRARY})
