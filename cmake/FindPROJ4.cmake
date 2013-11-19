#
# Find module for PROJ4
#
# The following variables will guide the find:
#   PROJ4_ROOT        - Set to the install prefix of the PROJ4 library.
#   PROJ4_DIR         - Find the package based on this path to the package's
#                       CMake configuration file. This supersedes the use of
#                       PROJ4_ROOT.
#
# The following variables will be set:
#   PROJ4_FOUND       - Set to TRUE if PROJ4 could be found.
#   PROJ4_INCLUDE_DIR - Path to the PROJ4 header files directory
#   PROJ4_LIBRARY     - The full path to the PROJ4 library.
#

if(PROJ4_DIR)
  find_package(PROJ4 NO_MODULE)
elseif(NOT PROJ4_FOUND)
  if(PROJ4_ROOT)
    set(_CMAKE_FIND_ROOT_PATH ${CMAKE_FIND_ROOT_PATH})
    set(CMAKE_FIND_ROOT_PATH ${PROJ4_ROOT})
    set(_PROJ4_ROOT_OPTS ONLY_CMAKE_FIND_ROOT_PATH)
  endif()

  find_path(PROJ4_INCLUDE_DIR "proj_api.h"
    ${_PROJ4_ROOT_OPTS}
    DOC "Path to the root directory containing the PROJ4 header file."
    )

  # If the header file was found, use that path as a introspective hint for finding the library
  if(PROJ4_INCLUDE_DIR AND (IS_DIRECTORY "${PROJ4_INCLUDE_DIR}"))
    #message("Introspecting library dir from include dir: '${PROJ4_INCLUDE_DIR}'")
    get_filename_component(proj4_lib_dir "${PROJ4_INCLUDE_DIR}/../lib" ABSOLUTE)
    #message("Introspected library dir: '${proj4_lib_dir}'")
    set(_PROJ4_LIB_PATH_HINT HINTS "${proj4_lib_dir}" "${proj4_lib_dir}64" NO_CMAKE_SYSTEM_PATH)
  endif()

  find_library(PROJ4_LIBRARY proj
    ${_PROJ4_LIB_PATH_HINT}
    ${_PROJ4_ROOT_OPTS}
    DOC "Path to the PROJ4 library."
    )

  if(PROJ4_ROOT)
    set(CMAKE_FIND_ROOT_PATH ${_CMAKE_FIND_ROOT_PATH})
  endif()

  include(FindPackageHandleStandardArgs)
  find_package_handle_standard_args(PROJ4 DEFAULT_MSG
    PROJ4_INCLUDE_DIR PROJ4_LIBRARY
    )
endif()
