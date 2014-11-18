#
# Helper functions for CMake configuring files
#
# Variables that affect behavior:
#
#   no_configure_target
#       If defined, configuration actions do not add explicit build targets.
#
include(CMakeParseArguments)

# Top level configuration target
add_custom_target(configure ALL)

#+
# Configure the given sourcefile to the given destfile
#
#   maptk_configure_file( name sourcefile destfile [var1 [var2 ...]]
#                         [DEPENDS ...] )
#
# Configure a sourcefile with a given name into the given destfile. Only the
# given variables (var1, var2, etc.) will be considered for replacement during
# configuration.
#
# This functions by generating custom configuration files for each call that
# controlls the configuration. Generated files are marked for cleaning.
#
# If ``no_configure_target`` is NOT set, this creates a target of the form
# ``configure-<name>`` for this configuration step.
#
# Additional configuration dependencies may be set with the DEPENDS and are
# passed to the underlying ``add_custom_command``.
#
# The special symbols ``__OUTPUT_PATH__``, ``__TEMP_PATH__``, and
# ``__SOURCE_PATH__`` are reserved by this method for additional configuration
# purposes, so don't use them as configuration variables in the file you are
# trying to configure.
#-
function(maptk_configure_file name source dest)
  set(multiValueArgs DEPENDS)
  cmake_parse_arguments(mcf "" "" "${multiValueArgs}" ${ARGN})

  message(STATUS "[configure-${name}] Creating configure command")

  set(gen_command_args)
  foreach(arg IN LISTS mcf_UNPARSED_ARGUMENTS)
    set(gen_command_args
      ${gen_command_args}
      "-D${arg}=${${arg}}"
      )
  endforeach()
  set(temp_file "${CMAKE_CURRENT_BINARY_DIR}/configure.${name}.output")
  add_custom_command(
    OUTPUT  "${dest}"
    COMMAND "${CMAKE_COMMAND}"
            ${gen_command_args}
            "-D__SOURCE_PATH__:PATH=${source}"
            "-D__TEMP_PATH__:PATH=${temp_file}"
            "-D__OUTPUT_PATH__:PATH=${dest}"
            -P "${MAPTK_SOURCE_DIR}/CMake/tools/maptk-configure-helper.cmake"
    DEPENDS
            "${source}" ${mcf_DEPENDS}
    WORKING_DIRECTORY
            "${CMAKE_CURRENT_BINARY_DIR}"
    COMMENT "Configuring ${name} file \"${source}\" -> \"${dest}\""
    )
  # also clean the intermediate generated file
  set_property(DIRECTORY APPEND PROPERTY
    ADDITIONAL_MAKE_CLEAN_FILES "${temp_file}"
    )

  # This passes if not defined or a false-evaluating value
  if(NOT no_configure_target)
    add_custom_target(configure-${name}
      DEPENDS "${dest}"
      SOURCES "${source}"   # Addding source for IDE purposes
      )
    source_group("Configured Files"
      FILES "${source}"
      )
    add_dependencies(configure
      configure-${name}
      )
  endif()
endfunction()
