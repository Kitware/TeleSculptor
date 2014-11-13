#
# MAPTK Target creation and installation support
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
#
#   library_subdir
#     If set, library targets will be placed into the directory with this
#     as a suffix. This is necessary due to the way some systems use
#     CMAKE_BUILD_TYPE as a directory in the output path.
#
include(CMakeParseArguments)

# Global collection variables
define_property(GLOBAL PROPERTY maptk_export_targets
  BRIEF_DOCS "Targets exported by MAPTK"
  FULL_DOCS "List of MAPTK targets to be exported in build and install trees."
  )
define_property(GLOBAL PROPERTY maptk_libraries
  BRIEF_DOCS "Libraries build as part of MAPTK"
  FULL_DOCS "List of static/shared libraries build by MAPTK"
  )


#+
# Helper function to manage export string string generation and the no_export
# flag.
#
# Sets the variable "exports" which should be expanded into the install
# command.
#-
function(_maptk_export name)
  set(exports)
  if(no_export)
    return()
  endif()
  set(exports
    EXPORT ${maptk_export_name}
    PARENT_SCOPE
    )
  set_property(GLOBAL APPEND PROPERTY maptk_export_targets ${name})
endfunction()

function(_maptk_compile_pic name)
  message(STATUS "Adding PIC flag to target: ${name}")
  if (CMAKE_VERSION VERSION_GREATER "2.8.9")
    set_target_properties("${name}"
      PROPERTIES
        POSITION_INDEPENDENT_CODE TRUE
      )
  elseif(NOT MSVC)
    set_target_properties("${name}"
      PROPERTIES
        COMPILE_FLAGS "-fPIC"
      )
  endif()
endfunction()

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
# Add an executable to MAPTK
#
#   maptk_add_executable(name [args...])
#
# All args given to this function are passed to CMake's add_executable(...)
# function after providing the name, so refer to CMake's documentation for
# additional valid arguments.
#
# This function will add the executable to the set of targets to be exported
# unless ``no_export`` was set.
#-
function(maptk_add_executable name)
  add_executable(${name} ${ARGN})
  set_target_properties(${name}
    PROPERTIES
      RUNTIME_OUTPUT_DIRECTORY "${MAPTK_BINARY_DIR}/bin"
    )

  if(NOT component)
    set(component runtime)
  endif()

  _maptk_export(${name})
  maptk_install(
    TARGETS     ${name}
    ${exports}
    DESTINATION bin
    COMPONENT   ${component}
    )
endfunction()

#+
# Add a library to MAPTK
#
#   maptk_add_library(name [args...])
#
# Remaining arguments passed to this function are given to the underlying
# add_library call, so refer to CMake documentation for additional arguments.
#
# Library version will be set to that of the current MAPTK version.
# Additionally defines the symbol "MAKE_<cname>_LIB" where ``cname`` is the
# ``name`` capitolized.
#
# This function will add the library to the set of targets to be exported
# unless ``no_export`` was set.
#-
function(maptk_add_library name)
  string(TOUPPER "${name}" upper_name)
  message(STATUS "Making library \"${name}\" with defined symbol \"MAKE_${upper_name}_LIB\"")

  add_library("${name}" ${ARGN})
  set_target_properties("${name}"
    PROPERTIES
      ARCHIVE_OUTPUT_DIRECTORY "${MAPTK_BINARY_DIR}/lib${library_subdir}"
      LIBRARY_OUTPUT_DIRECTORY "${MAPTK_BINARY_DIR}/lib${library_subdir}"
      RUNTIME_OUTPUT_DIRECTORY "${MAPTK_BINARY_DIR}/bin${library_subdir}"
      VERSION                  ${MAPTK_VERSION}
      SOVERSION                0
      DEFINE_SYMBOL            MAKE_${upper_name}_LIB
    )

  add_dependencies("${name}"
    configure-config.h
    configure-modules.h
    )

  foreach(config IN LISTS CMAKE_CONFIGURATION_TYPES)
    string(TOUPPER "${config}" upper_config)
    set_target_properties("${name}"
      PROPERTIES
        "ARCHIVE_OUTPUT_DIRECTORY_${upper_config}" "${MAPTK_BINARY_DIR}/lib/${config}${library_subdir}"
        "LIBRARY_OUTPUT_DIRECTORY_${upper_config}" "${MAPTK_BINARY_DIR}/lib/${config}${library_subdir}"
        "RUNTIME_OUTPUT_DIRECTORY_${upper_config}" "${MAPTK_BINARY_DIR}/bin/${config}${library_subdir}"
      )
  endforeach()

  if(NOT component)
    set(component runtime)
  endif()

  get_target_property(target_type "${name}" TYPE)
  if (target_type STREQUAL "STATIC_LIBRARY")
    _maptk_compile_pic("${name}")
  endif()

  _maptk_export(${name})
  # MATPK_LIB_SUFFIX should only apply to installation location, not the build
  # locations that properties above this point pertain to.
  maptk_install(
    TARGETS             "${name}"
    ${exports}
    ARCHIVE DESTINATION lib${MAPTK_LIB_SUFFIX}${library_subdir}
    LIBRARY DESTINATION lib${MAPTK_LIB_SUFFIX}${library_subdir}
    RUNTIME DESTINATION bin${library_subdir}
    COMPONENT           ${component}
    )

  set_property(GLOBAL APPEND PROPERTY maptk_libraries ${name})
endfunction()

#+
#   maptk_export_targets(file [APPEND])
#
# Export all recorded MAPTK targets to the given file in the build tree. If
# there are no targets recorded, this is a no-op. APPEND may be give to tell
# us to append to the given file instead of overwriting it.
#-
function(maptk_export_targets file)
  get_property(export_targets GLOBAL PROPERTY maptk_export_targets)
  export(
    TARGETS ${export_targets}
    ${ARGN}
    FILE "${file}"
    )
  #message(STATUS "Adding to file to clean: ${file}")
  #set_directory_properties(
  #  PROPERTIES
  #    ADDITIONAL_MAKE_CLEAN_FILES "${file}"
  #  )
endfunction()

#+
#   maptk_install_headers(header1 [header2 ...] [SUBDIR dir])
#
# Install MAPTK public header files to include/maptk.
#
# A SUBDIR may be provided in order to place the header files in a
# subdirectory under that. This path must be relative.
#-
function(maptk_install_headers)
  set(oneValueArgs SUBDIR)
  cmake_parse_arguments(mih "" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})
  #maptk_install(
  #  FILES       ${mih_UNPARSED_ARGUMENTS}
  #  DESTINATION "include/maptk/${mih_SUBDIR}"
  #  )
  foreach(header IN LISTS mih_UNPARSED_ARGUMENTS)
    get_filename_component(H_SUBDIR "${header}" PATH)
    maptk_install(
      FILES       "${header}"
      DESTINATION "include/maptk/${mih_SUBDIR}/${H_SUBDIR}"
      )
  endforeach()
endfunction()

#+
# Add files to the private header source group
#
#   maptk_private_header_group(file1 [file2 ...])
#
#-
function(maptk_private_header_group)
  source_group("Header Files\\Private"
    ${ARGN}
    )
endfunction()

#+
# Add files to the private template group
#
#   maptk_private_template_group(file1 [file2 ...])
#
#-
function(maptk_private_template_group)
  source_group("Template Files\\Private"
    ${ARGN}
    )
endfunction()
