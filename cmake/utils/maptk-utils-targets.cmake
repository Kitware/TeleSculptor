#
# MapTK Target creation and installation support
#
# Variables that affect behavior of functions:
#
#   no_export
#       if set, target will not be exported.
#
#   no_install
#       If set, target will not be installed.
#

#+
# Wrapper around install(...) that catches ``no_install`` if set
#
#   maptk_install([args])
#
# All args given to this function are passed directly to install(...). See
# CMake documentation for install(...) usage.
#-
function(maptk_install)
  if(no_install)
    return()
  endif()

  install(${ARGN})
endfunction()
