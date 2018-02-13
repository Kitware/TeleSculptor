file(GLOB config_files *.conf *.gz)

# Ensure target directory exists
set(out_dir "${MAPTK_BINARY_DIR}/share/maptk/${MAPTK_VERSION}/config")
add_custom_command(
  OUTPUT "${out_dir}"
  COMMAND "${CMAKE_COMMAND}" -E make_directory "${out_dir}"
  )

# Set up meta-target
add_custom_target(maptk-data-config ALL DEPENDS "${out_dir}")

# Add targets to copy config files
foreach(file ${config_files})
  get_filename_component(basename ${file} NAME_WE)
  get_filename_component(filename ${file} NAME)
  set(target maptk-data-config-${basename})
  kwiver_configure_file(${target}
    "${CMAKE_CURRENT_SOURCE_DIR}/${filename}"
    "${out_dir}/${filename}"
    )
  add_dependencies(maptk-data-config configure-${target})
endforeach()

# Install config files
kwiver_install(
  FILES ${config_files}
  COMPONENT runtime
  DESTINATION share/maptk/${MAPTK_VERSION}/config
  )
