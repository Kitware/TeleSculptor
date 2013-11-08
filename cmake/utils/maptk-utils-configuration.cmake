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

# Helper function for creating configuration scripts
#
# Required variables to be defined:
#   ``generated_configure_script``
#       - Path of the file to generate
#
function(_maptk_configure_file name source dest)
  file(WRITE "${generated_configure_script}"
    "# Configuring file \"${source}\" -> \"${dest}\"\n"
    )

  # Set each argument given, along with its value, to the generated file.
  foreach(arg IN LISTS ARGN)

    file(APPEND "${generated_configure_script}"
      "set(${arg} \"${${arg}}\")\n"
      )
  endforeach()

  file(APPEND "${generated_configure_script}" "
configure_file(
  \"${source}\"
  \"${dest}\"
  @ONLY)\n"
  )

  set(clean_files
    "${dest}"
    "${generated_configure_script}"
    )
  set_directory_properties(
    PROPERTIES
      ADDITIONAL_MAKE_CLEAN_FILES "${clean_files}"
    )
endfunction()

#
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
function(maptk_configure_file name source dest)
  message(STATUS "[configure-${name}] Configuring '${source}' -> '${dest}'")
  message(STATUS "[configure-${name}] ARGN: '${ARGN}'")

  set(generated_configure_script
    "${CMAKE_CURRENT_BINARY_DIR}/configure.${name}.cmake"
    )

  # Generated configure script
  _maptk_configure_file("${name}" "${source}" "${dest}" ${ARGN})

  add_custom_command(
    OUTPUT  "${dest}"
    COMMAND "${CMAKE_COMMAND}" -P "${generated_configure_script}"
    MAIN_DEPENDENCY
            "${source}"
    DEPENDS "${source}"
            "${generated_configure_script}"
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
