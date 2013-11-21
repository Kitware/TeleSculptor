#
# Take in a number of arguments and special arguments ``__OUTPUT_PATH__``,
# ``__SOURCE_PATH__`` and ``__TEMP_PATH__``
#

message(STATUS "Source Path       : '${__SOURCE_PATH__}'")
message(STATUS "Intermediate Path : '${__TEMP_PATH__}'")
message(STATUS "Output Path       : '${__OUTPUT_PATH__}'")
if(NOT EXISTS "${__SOURCE_PATH__}")
  message(FATAL_ERROR "Source file for configuration did not exist! -> ${__SOURCE_PATH__}")
endif()

# There are TWO configures here on purpose. The second configure containing
# the COPYONLY flag, only copies the file if the source and dest file are
# different (equivalent to ``cmake -E copy_if_different``). This helps prevent
# files from be touched during a forced configuration when none of the
# contained information changed (prevents rebuilding of dependant targets).
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
