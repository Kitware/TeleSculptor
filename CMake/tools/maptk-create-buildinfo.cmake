# Inputs:
#   GIT_EXECUTABLE    - Path to git executable
#   SOURCE_DIR        - Path to MAP-Tk source directory
#   BINARY_DIR        - Path to MAP-Tk build directory
#   COMPILER_NAME     - Name of the compiler to write to the build info file
#   COMPILER_VERSION  - Version of the compiler to write to the build info file

set(INPUT_FILE "${SOURCE_DIR}/BUILDINFO.in")
set(OUTPUT_FILE "${BINARY_DIR}/BUILDINFO")

set(TEMP_DIR "${BINARY_DIR}/CMakeFiles/buildinfo.dir")
make_directory(${TEMP_DIR})

###############################################################################

#BEGIN helper functions

#------------------------------------------------------------------------------
macro(git OUT)
  execute_process(
    COMMAND "${GIT_EXECUTABLE}" ${ARGN}
    WORKING_DIRECTORY "${SOURCE_DIR}"
    OUTPUT_VARIABLE ${OUT}
    OUTPUT_STRIP_TRAILING_WHITESPACE
    RESULT_VARIABLE _git_result
  )
  if (NOT _git_result EQUAL 0)
    file(REMOVE "${OUTPUT_FILE}")
    message(FATAL_ERROR "Execution of git subprocess failed")
  endif()
endmacro()

#END helper functions

###############################################################################

# Get SHA of current branch
git(SHA rev-parse HEAD)

# Get SHA of local modifications
#
# This is expensive, but CMake can't grok NUL-separated strings, so we can't
# reliably compare the list of locally modified files to the last time we
# updated the build information
git(DIFF_TEXT diff HEAD --)
string(SHA1 LOCAL_DIFF_SHA "${DIFF_TEXT}")

# Determine if we need to do anything
set(OUTDATED TRUE)
if (EXISTS "${TEMP_DIR}/HEAD" AND EXISTS "${TEMP_DIR}/DIFF")
  set(OUTDATED FALSE)

  # Test if branch has changed
  file(READ "${TEMP_DIR}/HEAD" OLD_SHA)
  if (NOT SHA STREQUAL OLD_SHA)
    set(OUTDATED TRUE)
  endif()

  # Test if list of local modifications are different
  file(READ "${TEMP_DIR}/DIFF" OLD_DIFF_SHA)
  if (NOT LOCAL_DIFF_SHA STREQUAL OLD_DIFF_SHA)
    set (OUTDATED TRUE)
  endif()
endif()

# Check if we have anything to do, also forcing an update if anything in the
# CMake cache has changed (this should catch things like changing the flags,
# using an external library from a different location, etc.)
if (NOT OUTDATED AND
    NOT "${BINARY_DIR}/CMakeCache.txt" IS_NEWER_THAN "${OUTPUT_FILE}")
  return()
endif()

# Write status files for future use
file(WRITE "${TEMP_DIR}/HEAD" "${SHA}")
file(WRITE "${TEMP_DIR}/DIFF" "${LOCAL_DIFF_SHA}")

# Get human-readable list of locally modified files
git(LOCAL_CHANGED_FILES diff --name-status)

# Generate remaining replacement texts
if (LOCAL_CHANGED_FILES STREQUAL "")
  set(BEGIN_LOCAL_CHANGES "<!--")
  set(END_LOCAL_CHANGES "-->")
  set(LOCAL_DIFF_SHA "(n/a)")
else()
  set(BEGIN_LOCAL_CHANGES "")
  set(END_LOCAL_CHANGES "")
endif()

set(BUILD_SHA "${SHA}")
string(TIMESTAMP BUILD_DATE UTC)

# Generate build information
configure_file("${INPUT_FILE}" "${OUTPUT_FILE}" @ONLY)
