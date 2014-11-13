#
# Find module for PROJ
#
# The following variables will guide the find:
#   PROJ_ROOT        - Set to the install prefix of the PROJ library.
#   PROJ_DIR         - Find the package based on this path to the package's
#                       CMake configuration file. This supersedes the use of
#                       PROJ_ROOT.
#
# The following variables will be set:
#   PROJ_FOUND       - Set to TRUE if PROJ could be found.
#   PROJ_INCLUDE_DIR - Path to the PROJ header files directory
#   PROJ_LIBRARY     - The full path to the PROJ library.
#

if(PROJ_DIR)
  find_package(PROJ NO_MODULE)
elseif(NOT PROJ_FOUND)
  include(CommonFindMacros)

  setup_find_root_context(PROJ)

  find_path(PROJ_INCLUDE_DIR "proj_api.h"
    ${PROJ_FIND_OPTS}
    DOC "Path to the root directory containing the PROJ header file."
    )

  # If the header file was found, use that path as a introspective hint for finding the library
  if(PROJ_INCLUDE_DIR AND (IS_DIRECTORY "${PROJ_INCLUDE_DIR}"))
    #message("Introspecting library dir from include dir: '${PROJ_INCLUDE_DIR}'")
    get_filename_component(proj_lib_dir "${PROJ_INCLUDE_DIR}/../lib" ABSOLUTE)
    #message("Introspected library dir: '${proj_lib_dir}'")
    set(_PROJ_LIB_PATH_HINT HINTS "${proj_lib_dir}" "${proj_lib_dir}64" NO_CMAKE_SYSTEM_PATH)
  endif()

  find_library(PROJ_LIBRARY proj
    ${_PROJ_LIB_PATH_HINT}
    ${PROJ_FIND_OPTS}
    DOC "Path to the PROJ library."
    )

  restore_find_root_context(PROJ)

  include(FindPackageHandleStandardArgs)
  find_package_handle_standard_args(PROJ DEFAULT_MSG
    PROJ_INCLUDE_DIR PROJ_LIBRARY
    )
endif()
