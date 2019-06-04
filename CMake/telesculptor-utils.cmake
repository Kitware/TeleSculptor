#
# TeleSculptor CMake utilities entry point
#

# Pin the current directory as the root directory for TeleSculptor utility files
set(TELESCULPTOR_UTIL_ROOT "${CMAKE_CURRENT_LIST_DIR}")

include("${CMAKE_CURRENT_LIST_DIR}/utils/telesculptor-utils-buildinfo.cmake")
