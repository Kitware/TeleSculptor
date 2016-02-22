#
# MAPTK CMake utilities entry point
#

# Pin the current directory as the root directory for MAPTK utility files
set(MAPTK_UTIL_ROOT "${CMAKE_CURRENT_LIST_DIR}")

include("${CMAKE_CURRENT_LIST_DIR}/utils/maptk-utils-buildinfo.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/utils/maptk-utils-targets.cmake")
