###
# Find kwiver module
find_package( kwiver REQUIRED )
include_directories( ${KWIVER_INCLUDE_DIRS} )
list(INSERT CMAKE_MODULE_PATH 0 "${KWIVER_CMAKE_DIR}" )

# If kwiver provides a Fletch directory, find Fletch as well
if ( IS_DIRECTORY ${fletch_DIR} )
  find_package( fletch NO_MODULE )
endif()

# KWIVER_LIBRARIES
