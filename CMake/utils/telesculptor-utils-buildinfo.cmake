find_package(Git REQUIRED)

add_custom_target(buildinfo
  COMMAND ${CMAKE_COMMAND}
          -DGIT_EXECUTABLE:FILEPATH="${GIT_EXECUTABLE}"
          -DSOURCE_DIR:PATH="${TELESCULPTOR_SOURCE_DIR}"
          -DBINARY_DIR:PATH="${TELESCULPTOR_BINARY_DIR}"
          -DCOMPILER_NAME:STRING="${CMAKE_CXX_COMPILER_ID}"
          -DCOMPILER_VERSION:STRING="${CMAKE_CXX_COMPILER_VERSION}"
          -P "${TELESCULPTOR_SOURCE_DIR}/CMake/tools/telesculptor-create-buildinfo.cmake"
  WORKING_DIRECTORY "${TELESCULPTOR_SOURCE_DIR}"
)

function(telesculptor_use_appdata)
  foreach(FILE ${ARGN})
    message(STATUS "File: '${FILE}'")
    if (FILE STREQUAL "BUILDINFO")
      add_custom_command(
        OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/BUILDINFO"
               "${CMAKE_CURRENT_BINARY_DIR}/BUILDINFO.phony"
        DEPENDS buildinfo
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${TELESCULPTOR_BINARY_DIR}/BUILDINFO"
                "${CMAKE_CURRENT_BINARY_DIR}/BUILDINFO"
      )
    else()
      add_custom_command(
        OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/${FILE}"
        DEPENDS "${TELESCULPTOR_SOURCE_DIR}/${FILE}"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${TELESCULPTOR_SOURCE_DIR}/${FILE}"
                "${CMAKE_CURRENT_BINARY_DIR}/${FILE}"
      )
    endif()
  endforeach()
endfunction()
