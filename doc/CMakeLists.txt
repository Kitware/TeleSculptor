find_package(Sphinx REQUIRED)

set(manual_dir "${CMAKE_BINARY_DIR}/doc/user")
add_custom_command(
  OUTPUT "${manual_dir}"
  COMMAND "${CMAKE_COMMAND}" -E make_directory "${manual_dir}"
)

add_custom_target(manuals ALL
  COMMAND ${SPHINX_EXECUTABLE}
    -b html
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${manual_dir}
  DEPENDS ${manual_dir}
  )

set_property(
  TARGET manuals
  PROPERTY ADDITIONAL_CLEAN_FILES ${manual_dir}
  )

kwiver_install(
  DIRECTORY   "${manual_dir}"
  DESTINATION "share/doc/telesculptor-${TeleSculptor_VERSION}"
  COMPONENT   documentation
)
