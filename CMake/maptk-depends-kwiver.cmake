###
# Find kwiver module
find_package( kwiver REQUIRED )
include_directories( ${KWIVER_INCLUDE_DIRS} )
list(INSERT CMAKE_MODULE_PATH 0 "${KWIVER_CMAKE_DIR}" )

# KWIVER_LIBRARIES
