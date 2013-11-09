#
# MapTK Target creation and installation support
#
# Variables that affect behavior of functions:
#
#   no_export
#       if set, target will not be exported.
#
#   no_install
#       If set, target will not be installed.
#
#   component
#     If set, the target will not be installed under this component (the
#     default is 'runtime').

# Global collection variables
set(__maptk_export_targets
  CACHE INTERNAL "Targets exported by MapTK"
  )

#+
# Wrapper around install(...) that catches ``no_install`` if set
#
#   maptk_install([args])
#
# All args given to this function are passed directly to install(...),
# provided ``no_install`` is not set. See CMake documentation for
# install(...) usage.
#-
function(maptk_install)
  if(no_install)
    return()
  endif()

  install(${ARGN})
endfunction()

#+
# Add a library to MapTK
#
#   maptk_add_library(name [args...])
#
# Remaining arguments passed to this function are given to the underlying
# add_library call, so refer to CMake documentation for additional arguments.
#
# This function will add the library to the set of targets to be exported
# unless ``no_export`` was set.
#-
function(maptk_add_library name)
  add_library("${name}" ${ARGN})
  set_target_properties("${name}"
    PROPERTIES
      ARCHIVE_OUTPUT_DIRECTORY "${MAPTK_BINARY_DIR}/lib"
      LIBRARY_OUTPUT_DIRECTORY "${MAPTK_BINARY_DIR}/lib"
      RUNTIME_OUTPUT_DIRECTORY "${MAPTK_BINARY_DIR}/bin"
    )

  add_dependencies("${name}"
    configure-config.h
    )

  foreach(config IN LISTS CMAKE_CONFIGURATION_TYPES)
    string(TOUPPER "${config}" upper_config)
    set_target_properties("${name}"
      PROPERTIES
        "ARCHIVE_OUTPUT_DIRECTORY_${upper_config}" "${MAPTK_BINARY_DIR}/lib/${config}"
        "LIBRARY_OUTPUT_DIRECTORY_${upper_config}" "${MAPTK_BINARY_DIR}/lib/${config}"
        "RUNTIME_OUTPUT_DIRECTORY_${upper_config}" "${MAPTK_BINARY_DIR}/bin/${config}"
      )
  endforeach()

  if(NOT component)
    set(component runtime)
  endif()

  _maptk_export(${name})
  maptk_install(
    TARGETS             "${name}"
    ${exports}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    COMPONENT           ${component}
    )
endfunction()

#+
# Helper macro to manage export string string generation and the no_export
# flag.
#
# Sets the variable "exports" which should be expanded into the install
# command.
#-
macro(_maptk_export name)
  set(exports)
  if(no_export)
    return()
  endif()
  set(exports
    EXPORT ${maptk_export_name}
    )
  set(__maptk_export_targets
    ${__maptk_export_targets}
    ${name}
    CACHE INTERNAL "Targets exported by MapTK"
    )
endmacro()

#+
#   maptk_export_targets(file [APPEND])
#
# Export all recorded MapTK targets to the given file in the build tree. If
# there are no targets recorded, this is a no-op. APPEND may be give to tell
# us to append to the given file instead of overwriting it.
#-
function(maptk_export_targets file)
  export(
    TARGETS ${__maptk_export_targets}
    ${ARGN}
    FILE "${file}"
    )
  #message(STATUS "Adding to file to clean: ${file}")
  #set_directory_properties(
  #  PROPERTIES
  #    ADDITIONAL_MAKE_CLEAN_FILES "${file}"
  #  )
endfunction()
