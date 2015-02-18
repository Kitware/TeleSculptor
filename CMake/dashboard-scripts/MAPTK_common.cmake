# MAPTK Common Dashboard Script
#
# This script contains basic dashboard driver code common to all clients for
# the MAPTK project.
#
# Put this script in a directory such as "~/Dashboards/Scripts" or
# "C:\Dashboards\Scripts". Create a file next to this script, say
# "my_dashboard.cmake", with code of the following form:
#
#   # Client maintainer: me@mydomain.net
#   set(CTEST_SITE "machine.site")
#   set(CTEST_BUILD_NAME "Platform-Compiler")
#   set(CTEST_CONFIGURATION_TYPE Debug)
#   set(CTEST_CMAKE_GENERATOR "Unix Makefiles")
#   include("${CTEST_SCRIPT_DIRECTORY}/MAPTK_common.cmake")
#
# Then run a scheduled task (cron job) with a command line such as:
#
#   ctest -S ~/Dashboards/Scripts/my_dashboard.cmake -V
#
# By default, the source and build trees will be placed in the path
# "../MyTests/" relative to your script location.
#
# The following variables may be set before including this script to configure
# it:
#
#   dashboard_model           = Nightly [default] | Experimental | Continuous
#   dashboard_root_name       = Change name of "MyTests" directory. This is
#                               ignored if CTEST_DASHBOARD_ROOT is set.
#   dashboard_source_name     = Name of source directory (default: MAPTK)
#   dashboard_binary_name     = Name of binary directory (default: MAPTK-build)
#   dashboard_cache           = Initial CMakeCache.txt file content.
#   dashboard_do_coverage     = True to enable coverage.
#   dashboard_do_memcheck     = True to enable memcheck.
#   dashboard_no_submit       = Skip result submission step.
#   CTEST_CHECKOUT_COMMAND    = Custom source tree checkout comamnd (primarilly
#                               for if the VCS is not git).
#   CTEST_BUILD_FLAGS         = build too arguments (ex: -j2)
#   CTEST_DASHBOARD_ROOT      = Where to put source and build trees
#   CTEST_TEST_CTEST          = Whether to run long CTestTest* tests.
#   CTEST_TEST_TIMEOUT        = Per-test timeout length.
#   CTEST_TEST_ARGS           = ctest_test args (ex: PARALLEL_LEVEL 4)
#   CMAKE_MAKE_PROGRAM        = Path to "make" tool to use when using makefile
#                               generator.
#   CTEST_COVERAGE_COMMAND    = Path to the coverage tool to use when testing
#                               coverage (dashboard_do_coverage).
#   CTEST_MEMORYCHECK_COMMAND = Path to the memory check too to use when
#                               testing memory usage (dashboard_do_memcheck).
#
# Options to configure Git:
#   CTEST_GIT_COMMAND         = Path to the git command-line client.
#   dashboard_git_url         = Custom git clone URL (defaults to KWSource).
#   dashboard_git_branch      = Custom remote branch to track (defaults to
#                               master). If this is not a valid for the set
#                               repository, the checkout script will silently
#                               fail, leading to an error in ctest_start when
#                               the source directory doesn't exist.
#   dashboard_git_crlf        = Value of core.autocrlf for repository.
#
# The following macros will be invoked before the corresponding step if they
# are defined:
#
#   dashboard_hook_init     = End of initialization, before loop
#   dashboard_hook_start    = Start of loop body, before ctest_start
#   dashboard_hook_started  = Start of loop body, after ctest_start
#   dashboard_hook_build    = Before ctest_build
#   dashboard_hook_test     = Before ctest_test
#   dashboard_hook_coverage = Before ctest_coverage, if enabled
#   dashboard_hook_memcheck = Before ctest_memcheck, if enabled
#   dashboard_hook_submit   = Before ctest_submit, if enabled
#   dashboard_hook_end      = End of body loop, after ctest_submit.
#
# For Makefile generators the script may be executed from as environment
# already configured to use the desired compilers. Alternatively, the
# environment may be set at the top of the script:
#
#   set(ENV{CC}   /path/to/cc)  # C compiler
#   set(ENV{CXX}  /path/to/cxx) # C++ compiler
#   set(ENV{FC}   /path/to/fc)  # Fortran Compiler (optional)
#   set(ENV{LD_LIBRARY_PATH} /path/to/vendor/lib) # (if necessary)
#

cmake_minimum_required(VERSION 2.8.2 FATAL_ERROR)

#-----------------------------------------------------------------------------
# CTest properties setup
#

set(CTEST_PROJECT_NAME MAPTK)
set(dashboard_user_home "$ENV{HOME}")

# Select the top dashboard directory
if(NOT DEFINED dashboard_root_name)
  set(dashboard_root_name "MyTests")
endif()
if(NOT DEFINED CTEST_DASHBOARD_ROOT)
  get_filename_component(CTEST_DASHBOARD_ROOT "${CTEST_SCRIPT_DIRECTORY}/../${dashboard_root_name}" ABSOLUTE)
endif()

# Select ctest model (Nightly | Experimental | Continuous)
if(NOT DEFINED dashboard_model)
  set(dashboard_model Nightly)
endif()
if(NOT "${dashboard_model}" MATCHES "^(Nightly|Experimental|Continuous)$")
  message(FATAL_ERROR "dashboard_model must be either Nightly, Experimental or Continuous.")
endif()

# Default to debug build configuration if one not provided
if(NOT DEFINED CTEST_CONFIGURATION_TYPE)
  set(CTEST_CONFIGURATION_TYPE Debug)
endif()

# Choosing CTest reporting mode
if(NOT "${CTEST_CMAKE_GENERATOR}" MATCHES "Make")
  set(CTEST_USE_LAUNCHERS 0)
elseif(NOT DEFINED CTEST_USE_LAUNCHERS)
  set(CTEST_USE_LAUNCHERS 1)
endif()

# Configuring testing
if(NOT DEFINED CTEST_TEST_CTEST)
  set(CTEST_TEST_CTEST 1)
endif()
if(NOT CTEST_TEST_TIMEOUT)
  set(CTEST_TEST_TIMEOUT 1500)
endif()

# Selecting Git source to use
if(NOT DEFINED dashboard_git_url)
  set(dashboard_git_url "https://github.com/Kitware/maptk.git")
endif()
if(NOT DEFINED dashboard_git_branch)
  set(dashboard_git_branch master)
endif()
if(NOT DEFINED dashboard_git_crlf)
  if(UNIX)
    set(dashboard_git_crlf false)
  else()
    set(dashboard_git_crlf true)
  endif()
endif()

# Look for Git command-line tool if not provided
if(NOT DEFINED CTEST_GIT_COMMAND)
  find_program(CTEST_GIT_COMMAND
    names git git.cmd
    PATH_SUFFIXES Git/cmd Git/bin
    )
endif()
if(NOT CTEST_GIT_COMMAND)
  message(FATAL_ERROR "CTEST_GIT_COMMAND not available!")
endif()

# Selecting source and binary tree directories
if(NOT DEFINED CTEST_SOURCE_DIRECTORY)
  if(DEFINED dashboard_source_name)
    set(CTEST_SOURCE_DIRECTORY "${CTEST_DASHBOARD_ROOT}/${dashboard_source_name}")
  else()
    set(CTEST_SOURCE_DIRECTORY "${CTEST_DASHBOARD_ROOT}/MAPTK")
  endif()
endif()
if(NOT DEFINED CTEST_BINARY_DIRECTORY)
  if(DEFINED dashboard_binary_name)
    set(CTEST_BINARY_DIRECTORY "${CTEST_DASHBOARD_ROOT}/${dashboard_binary_name}")
  else()
    set(CTEST_BINARY_DIRECTORY "${CTEST_DASHBOARD_ROOT}/MAPTK-build")
  endif()
endif()

# Delete source tree if not compatible with current VCS
if(EXISTS "${CTEST_SOURCE_DIRECTORY}")
  if(NOT EXISTS "${CTEST_SOURCE_DIRECTORY}/.git")
    set(vcs_refresh "because it is not managed by Git.")
  else()
    execute_process(
      COMMAND "${CTEST_GIT_COMMAND}" reset --hard
      WORKING_DIRECTORY "${CTEST_SOURCE_DIRECTORY}"
      OUTPUT_VARIABLE output
      ERROR_VARIABLE output
      RESULT_VARIABLE failed
      )
    if(failed)
      set(vcs_refresh "because its .git may be corrupted.")
    endif()
  endif()
  if(vcs_refresh)
    message("Deleting source tree\n  ${CTEST_SOURCE_DIRECTORY}\n${vcs_refresh}")
    file(REMOVE_RECURSE "${CTEST_SOURCE_DIRECTORY}")
  endif()
endif()

# Support initial checkout if necessary
if(NOT EXISTS "${CTEST_SOURCE_DIRECTORY}"
    AND NOT DEFINED CTEST_CHECKOUT_COMMAND)
  get_filename_component(_name "${CTEST_SOURCE_DIRECTORY}" NAME)
  execute_process(
    COMMAND "${CTEST_GIT_COMMAND}" --version
    OUTPUT_VARIABLE output
    )
  string(REGEX MATCH "[0-9]+\\.[0-9]+\\.[0-9]+(\\.[0-9]+(\\.g[0-9a-f]+)?)?" GIT_VERSION "${output}")
  if(NOT "${GIT_VERSION}" VERSION_LESS "1.6.5")
    set(git_branch_new "-b ${dashboard_git_branch}")
    set(git_branch_old)
  else()
    set(git_branch_new)
    set(git_branch_old "-b ${dashboard_git_branch} origin/${dashboard_git_branch}")
  endif()

  # generating initial checkout script
  set(ctest_checkout_script "${CTEST_DASHBOARD_ROOT}/${_name}-init.cmake")
  file(WRITE "${ctest_checkout_script}" "# git repo init script for ${_name}
execute_process(
  COMMAND \"${CTEST_GIT_COMMAND}\" clone -n ${git_branch_new} --
          \"${dashboard_git_url}\" \"${CTEST_SOURCE_DIRECTORY}\"
  )
if(EXISTS \"${CTEST_SOURCE_DIRECTORY}/.git\")
  execute_process(
    COMMAND \"${CTEST_GIT_COMMAND}\" config core.autocrlf ${dashboard_git_crlf}
    WORKING_DIRECTORY \"${CTEST_SOURCE_DIRECTORY}\"
    )
  execute_process(
    COMMAND \"${CTEST_GIT_COMMAND}\" checkout ${git_branch_old}
    WORKING_DIRECTORY \"${CTEST_SOURCE_DIRECTORY}\"
    )
endif()
")
  set(CTEST_CHECKOUT_COMMAND "\"${CMAKE_COMMAND}\" -P \"${ctest_checkout_script}\"")
endif()


#-----------------------------------------------------------------------------
# Setup and run Ctest
#

# Send main script as a note to the submission
list(APPEND CTEST_NOTES_FILES
  "${CTEST_SCRIPT_DIRECTORY}/${CTEST_SCRIPT_NAME}"
  "${CMAKE_CURRENT_LIST_FILE}"
  )

# Check required variables
foreach(req
    CTEST_CMAKE_GENERATOR
    CTEST_SITE
    CTEST_BUILD_NAME
    )
  if(NOT DEFINED ${req})
    message(FATAL_ERROR "The containing script must set ${req}")
  endif()
endforeach(req)

# Print summary information
set(vars "")
foreach(v
    CTEST_SITE
    CTEST_BUILD_NAME
    CTEST_SOURCE_DIRECTORY
    CTEST_BINARY_DIRECTORY
    CTEST_CMAKE_GENERATOR
    CTEST_CONFIGURATION_TYPE
    CTEST_GIT_COMMAND
    CTEST_CHECKOUT_COMMAND
    CTEST_CONFIGURE_COMMAND
    CTEST_SCRIPT_DIRECTORY
    CTEST_USE_LAUNCHERS
    )
  set(vars "${vars} ${v}=[${${v}}]\n")
endforeach(v)
message("Dashboard script configuration:\n${vars}\n")

# Avoid non-ascii characters in tool output
set(ENV{LC_ALL} C)

# Helper macro to write initial cache
macro(write_cache)
  set(cache_build_type "")
  set(cache_make_program "")
  if(CTEST_CMAKE_GENERATOR MATCHES "Make")
    set(cache_build_type "CMAKE_BUILD_TYPE:STRING=${CTEST_CONFIGURATION_TYPE}")
    if(CMAKE_MAKE_PROGRAM)
      set(cache_make_program "CMAKE_MAKE_PROGRAM:FILEPATH=${CMAKE_MAKE_PROGRAM}")
    endif()
  endif()
  file(WRITE "${CTEST_BINARY_DIRECTORY}/CMakeCache.txt" "
SITE:STRING=${CTEST_SITE}
BUILDNAME:STRING=${CTEST_BUILD_NAME}
CTEST_TEST_CTEST:BOOL=${CTEST_TEST_CTEST}
CTEST_USE_LAUNCHERS:BOOL=${CTEST_USE_LAUNCHERS}
DART_TESTING_TIMEOUT:STRING=${CTEST_TEST_TIMEOUT}
GIT_EXECUTABLE:FILEPATH=${CTEST_GIT_COMMAND}
${cache_build_type}
${cache_make_program}
${dashboard_cache}
")
endmacro(write_cache)

# Start with fresh build tree
file(MAKE_DIRECTORY "${CTEST_BINARY_DIRECTORY}")
if(NOT "${CTEST_SOURCE_DIRECTORY}" STREQUAL "${CTEST_BINARY_DIRECTORY}")
  ctest_empty_binary_directory("${CTEST_BINARY_DIRECTORY}")
endif()

set(dashboard_continuous 0)
if("${dashboard_model}" STREQUAL "Continuous")
  set(dashboard_continuous 1)
endif()

# CTest 2.6 crashes with message() after ctest_test
macro(safe_message)
  if(NOT "${CMAKE_VERSION}" VERSION_LESS 2.8 OR NOT safe_message_skip)
    message(${ARGN})
  endif()
endmacro(safe_message)

if(COMMAND dashboard_hook_init)
  dashboard_hook_init()
endif()

set(dashboard_done 0)
while(NOT dashboard_done)
  if(dashboard_continuous)
    set(START_TIME ${CTEST_ELAPSED_TIME})
  endif()
  set(ENV{HOME} "${dashboard_user_home}")

  # Start a new submission
  if(COMMAND dashboard_hook_start)
    dashboard_hook_start()
  endif()
  ctest_start(${dashboard_model})
  if(COMMAND dashboard_hook_started)
    dashboard_hook_started()
  endif()

  # Always build if the tree is fresh
  set(dashboard_fresh 0)
  if(NOT EXISTS "${CTEST_BINARY_DIRECTORY}/CMakeCache.txt")
    set(dashboard_fresh 1)
    safe_message("Starting fresh build...")
    write_cache()
  endif()

  # Look for updates
  ctest_update(RETURN_VALUE count)
  set(CTEST_CHECKOUT_COMMAND) # checkout on first iteration only
  safe_message("Found ${count} changed files")
  if(dashboard_fresh OR NOT dashboard_continuous OR count GREATER 0)
    ctest_configure()
    ctest_read_custom_files("${CTEST_BINARY_DIRECTORY}")

    if(COMMAND dashboard_hook_build)
      dashboard_hook_build()
    endif()
    ctest_build()

    if(COMMAND dashboard_hook_test)
      dashboard_hook_test()
    endif()
    ctest_test(${CTEST_TEST_ARGS})
    set(safe_message_skip 1)

    if(dashboard_do_coverage)
      if(COMMAND dashboard_hook_coverage)
        dashboard_hook_coverage()
      endif()
      ctest_coverage()
    endif()
    if(dashboard_do_memcheck)
      if(COMMAND dashboard_hook_memcheck)
        dashboard_hook_memcheck()
      endif()
      ctest_memcheck()
    endif()
    if(NOT dashboard_no_submit)
      if(COMMAND dashboard_hook_submit)
        dashboard_hook_submit()
      endif()
      ctest_submit()
    endif()

    if(COMMAND dashboard_hook_end)
      dashboard_hook_end()
    endif()
  endif()

  if(dashboard_continuous)
    # Delay until at least 5 minutes past START_TIME
    ctest_sleep(${START_TIME} 300 ${CTEST_ELAPSED_TIME})
    if(${CTEST_ELAPSED_TIME} GREATER 86400)
      set(dashboard_done 1)
    endif()
  else()
    set(dashboard_done 1)
  endif()
endwhile()
