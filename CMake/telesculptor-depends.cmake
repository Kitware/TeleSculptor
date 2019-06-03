###
# use kwiver/fletch/telesculptor util methods
#

message(STATUS "Looking for kwiver in : ${kwiver_DIR}")

find_package(kwiver REQUIRED)
include_directories(SYSTEM "${KWIVER_INCLUDE_DIRS}")
link_directories("${KWIVER_LIBRARY_DIRS}")
include( kwiver-cmake-future )
include( kwiver-utils )
include( kwiver-flags )
include( kwiver-configcheck )
if ( IS_DIRECTORY ${fletch_DIR} )
  find_package(fletch REQUIRED)
  message(STATUS "Using fletch from : ${fletch_DIR}")
endif()
include( telesculptor-utils ) # local utilities

