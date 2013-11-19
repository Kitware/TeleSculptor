#
# Take in a number of arguments and special arguments ``__OUTPUT_PATH__``,
# ``__SOURCE_PATH__`` and ``__TEMP_PATH__``
#

# Well this is redundant...
#foreach(i RANGE 1 ${CMAKE_ARGC})
#  #message(STATUS "arg: '${CMAKE_ARGV${i}}'")
#  string(REGEX MATCH "-D([a-zA-Z0-9_]*)=(.*)" re_match "${CMAKE_ARGV${i}}")
#  #message(STATUS "match 0: ${CMAKE_MATCH_0}")
#  #message(STATUS "match 1: ${CMAKE_MATCH_1}")
#  #message(STATUS "match 2: ${CMAKE_MATCH_2}")
#  if(NOT (   ("${CMAKE_MATCH_1}" STREQUAL "")
#          OR ("${CMAKE_MATCH_1}" STREQUAL "__OUTPUT_PATH__")
#          OR ("${CMAKE_MATCH_1}" STREQUAL "__SOURCE_PATH__")
#          OR ("${CMAKE_MATCH_1}" STREQUAL "__TEMP_PATH__")
#          ))
#    set(${CMAKE_MATCH_1} "${CMAKE_MATCH_2}")
#    message(STATUS "${CMAKE_MATCH_1} := \"${CMAKE_MATCH_2}\"")
#  endif()
#endforeach()

message(STATUS "Source Path       : '${__SOURCE_PATH__}'")
message(STATUS "Intermediate Path : '${__TEMP_PATH__}'")
message(STATUS "Output Path       : '${__OUTPUT_PATH__}'")
if(NOT EXISTS "${__SOURCE_PATH__}")
  message(FATAL_ERROR "Source file for configuration did not exist! -> ${__SOURCE_PATH__}")
endif()

# Performing a double configure. The second configure, the COPYONLY, only
# replaces the destination file if the contents differ.
configure_file(
  "${__SOURCE_PATH__}"
  "${__TEMP_PATH__}"
  @ONLY
  )
configure_file(
  "${__TEMP_PATH__}"
  "${__OUTPUT_PATH__}"
  COPYONLY
  )
