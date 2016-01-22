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
#   no_version
#       If set, the target will not have version information added to it.
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
define_property(GLOBAL PROPERTY maptk_plugin_libraries
  BRIEF_DOCS "Generated plugin libraries"
  FULL_DOCS "List of generated shared plugin module libraries"
  )

define_property(GLOBAL PROPERTY maptk_bundle_paths
  BRIEF_DOCS "Paths needed by fixup_bundle"
  FULL_DOCS "Paths needed to resolve needed libraries used by plugins when fixing the bundle"
  )

# Top-level target for plugin targets
add_custom_target( all-plugins )


#+
# Generate and add a plug-in library based on another library
#
#   maptk_create_plugin(base_lib [args ...])
#
# The given base library must link against the core maptk library and provide
# an implementation of the algorithm plugin interface class. If this has not
# been done an error will occur at link time stating that the required class
# symbol can not be found.
#
# This generates a small MODULE library that exposes the required C interface
# function to be picked up by the algorithm plugin manager. This library is set
# to install into the .../maptk subdirectory and adds a _plugin suffix to the
# base library name.
#
# Additional source files may be specified after the base library if the
# registration interface implementation is separate from the base library.
#
# Setting library_subdir or no_export before this function
# has no effect as they are manually specified within this function.
#-
function(maptk_create_plugin    base_lib)
  message( STATUS "Building plugin \"${base_lib}\"" )

  # Configure template cxx source file
  set(shell_source "${MAPTK_UTIL_ROOT}/templates/cxx/plugin_shell.cxx")

  # create module library given generated source, linked to given library
  set(library_subdir /maptk)
  set(no_version ON)
  set(no_export_header ON)

  kwiver_add_plugin( maptk-plugin-${base_lib}
    SOURCES  ${shell_source} ${ARGN}

    # Not adding link to known base MAPTK library because if the base_lib isn't
    # linking against it, its either doing something really complex or doing
    # something wrong (most likely the wrong).
    PRIVATE  ${base_lib}
    )

  # used by plugin shell
  target_compile_definitions( maptk-plugin-${base_lib}
    PRIVATE
    "MAPTK_PLUGIN_LIB_NAME=\"${base_lib}\""
    )

  set_target_properties( maptk-plugin-${base_lib}
    PROPERTIES
    OUTPUT_NAME   ${base_lib}_plugin
    INSTALL_RPATH "\$ORIGIN/../../lib:\$ORIGIN/"
    )

  add_dependencies( all-plugins maptk-plugin-${base_lib} )

  # For each library linked to the base library, add the path to the library
  # to a list of paths to search later during fixup_bundle.
  get_target_property(deps ${base_lib} LINK_LIBRARIES)
  foreach( dep ${deps} )
    if(TARGET "${dep}")
      list(APPEND PLUGIN_BUNDLE_PATHS $<TARGET_FILE_DIR:${dep}>)
    elseif(EXISTS "${dep}")
      get_filename_component(dep_dir "${dep}" DIRECTORY)
      list(APPEND PLUGIN_BUNDLE_PATHS ${dep_dir})
    endif()
  endforeach()

  # Add to global collection variables
  set_property(GLOBAL APPEND
    PROPERTY maptk_plugin_libraries    maptk-plugin-${base_lib}
    )
  set_property(GLOBAL APPEND
    PROPERTY maptk_bundle_paths ${PLUGIN_BUNDLE_PATHS}
    )

endfunction()
