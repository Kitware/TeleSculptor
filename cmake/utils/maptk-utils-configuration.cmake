#
# Helper functions for CMake configuring files
#
# Variables that affect behavior:
#
#   no_configure_target
#       If defined, configuration actions do not add explicit build targets.
#

# Top level condiguration target
add_custom_target(configure ALL)

#+
# Configure the given sourcefile to the given destfile
#
#   maptk_configure_file(name sourcefile destfile [var1 [var2 ...]])
#
# Configure a sourcefile with a given name into the given destfile. Only the
# given variables (var1, var2, etc.) will be considered for replacement during
# configuration.
#
# This functions by generating custom configuration files for each call that
# controlls the configuration. Generated files are marked for cleaning.
#
# The special symbols ``__OUTPUT_PATH__`` and ``__SOURCE_PATH__`` are reserved
# by this method, so don't use them as configuration variables in the file you
# are trying to configure.
#-
function(maptk_configure_file name source dest)
  message(STATUS "[configure-${name}] Creating configure command")

  set(gen_command_args)
  foreach(arg IN LISTS ARGN)
    set(gen_command_args
      ${gen_command_args}
      "-D${arg}=${${arg}}"
      )
  endforeach()
  add_custom_command(
    OUTPUT  "${dest}"
    COMMAND "${CMAKE_COMMAND}"
            ${gen_command_args}
            "-D__SOURCE_PATH__:PATH=${source}"
            "-D__OUTPUT_PATH__:PATH=${dest}"
            -P "${MAPTK_SOURCE_DIR}/cmake/tools/maptk-configure-helper.cmake"
    MAIN_DEPENDENCY
            "${source}"
    WORKING_DIRECTORY
            "${CMAKE_CURRENT_BINARY_DIR}"
    COMMENT "Configuring ${name} file \"${source}\" -> \"${dest}\""
    )

  # This passes if not defined or a false-evaluating value
  if(NOT no_configure_target)
    add_custom_target(configure-${name} ${all}
      DEPENDS "${dest}"
      SOURCES "${source}"
      )
    source_group("Configured Files"
      FILES "${source}"
      )
    add_dependencies(configure
      configure-${name}
      )
  endif()
endfunction()
