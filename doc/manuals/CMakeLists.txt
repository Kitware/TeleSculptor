find_package(PythonInterp REQUIRED)
execute_process(
  COMMAND ${PYTHON_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/FindDocUtils.py

  RESULT_VARIABLE DOCUTILS_FOUND
)
if(NOT DOCUTILS_FOUND EQUAL 0)
  message(FATAL_ERROR
    "Failed to import docutils. Please ensure that docutils is installed"
    " and that ${PYTHON_EXECUTABLE} is able to find it"
    " (adjusting your PYTHONPATH if necessary)."
  )
endif()

set(configure_file_script
  "${visGUI_SOURCE_DIR}/CMake/configure-file-script.cmake"
)

set(stylesheet "${CMAKE_CURRENT_SOURCE_DIR}/default.css")

set(manual_dir "${CMAKE_BINARY_DIR}/doc/user")
add_custom_command(
  OUTPUT "${manual_dir}"
  COMMAND "${CMAKE_COMMAND}" -E make_directory "${manual_dir}"
)

set(create_manual_script "${CMAKE_CURRENT_BINARY_DIR}/create_manual.py")

kwiver_configure_file( create_manual.py
  "${CMAKE_CURRENT_SOURCE_DIR}/create_manual.py.in"
  "${create_manual_script}"
  PYTHON_EXECUTABLE
  stylesheet
  )

set(outputs)
foreach(man telesculptor)
  set(_in "${CMAKE_CURRENT_SOURCE_DIR}/${man}.rst")
  set(_out "${manual_dir}/${man}.html")
  add_custom_command(
    OUTPUT "${_out}"
    DEPENDS "${create_manual_script}" "${manual_dir}"
            "${_in}" ${stylesheet}
    COMMAND "${PYTHON_EXECUTABLE}" "${create_manual_script}"
      "${_in}"
      "${_out}"
      "${TELESCULPTOR_SOURCE_DIR}/gui/icons/16x16"
      "${TELESCULPTOR_SOURCE_DIR}/gui/icons/22x22"
  )
  list(APPEND outputs "${_out}")
endforeach()

add_custom_target(manuals ALL DEPENDS ${outputs})

kwiver_install(
  DIRECTORY   "${manual_dir}"
  DESTINATION "share/doc/telesculptor-${TeleSculptor_VERSION}"
  COMPONENT   documentation
)
