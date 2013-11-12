#
# Take in a number of arguments and special arguments ``__OUTPUT_PATH__`` and
# ``__SOURCE_PATH__``
#
foreach(i RANGE 1 ${CMAKE_ARGC})
  #message(STATUS "arg: '${CMAKE_ARGV${i}}'")
  string(REGEX MATCH "-D([a-z_A-Z0-9]*)=(.*)" re_match "${CMAKE_ARGV${i}}")
  #message(STATUS "match 0: ${CMAKE_MATCH_0}")
  #message(STATUS "match 1: ${CMAKE_MATCH_1}")
  #message(STATUS "match 2: ${CMAKE_MATCH_2}")
  if(NOT (   ("${CMAKE_MATCH_1}" STREQUAL "")
          OR ("${CMAKE_MATCH_1}" STREQUAL "__OUTPUT_PATH__")
          OR ("${CMAKE_MATCH_1}" STREQUAL "__SOURCE_PATH__")
          ))
    set(${CMAKE_MATCH_1} "${CMAKE_MATCH_2}")
    message(STATUS "${CMAKE_MATCH_1} := \"${CMAKE_MATCH_2}\"")
  endif()
endforeach()
message(STATUS "Source Path: '${__SOURCE_PATH__}'")
message(STATUS "Output Path: '${__OUTPUT_PATH__}'")

configure_file(
  "${__SOURCE_PATH__}"
  "${__OUTPUT_PATH__}"
  @ONLY
  )
