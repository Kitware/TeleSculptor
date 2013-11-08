#
# Utility macros and functions dealing with MapTK modules
#
include(CMakeParseArguments)

set(_maptk_modules_enabled
  CACHE INTERNAL "Active (turned on) maptk modules"
  )

#
# Add a directory as a named module.
#
#   maptk_add_module(name directory [OPTIONAL]
#                    [DEPENDS module1 [module2 ...]])
#
# Register a module to be built. This module must be named in order to
# associate references. Names must be unique across added modules. A module
# may be labeled as OPTIONAL, creating an ENABLE/DISABLE flag for the module
# (disabled by default). Modules may also depend on other modules. This does
# not affect build dependencies, but ensures that the listed modules are
# enabled.
#
function(maptk_add_module name directory)
  # Parsing args
  set(options OPTIONAL)
  set(multiValueArgs DEPENDS)
  cmake_parse_arguments(module "${options}" "" "${multiValueArgs}" ${ARGN})

  if(NOT "${module_UNPARSED_ARGUMENTS}" STREQUAL "")
    message(WARNING "maptk_add_module called with unparsed args! (${module_UNPARSED_ARGUMENTS})")
  endif()

  # Check that name is unique among modules registered so far
  list(FIND _maptk_modules_enabled "${name}" duplicate_index)
  if(NOT duplicate_index EQUAL -1)
    message(FATAL_ERROR "Attempted to register duplicate module '${name}'!")
  endif()

  #TODO: Check that each dep is in module enable list
  foreach(dep IN LISTS module_DEPENDS)
    list(FIND _maptk_modules_enabled ${dep} dep_index)
    if(dep_index EQUAL -1)
      message(SEND_ERROR "Module '${name}' missing module dependency: '${dep}'")
      set(module_error TRUE)
    endif()
  endforeach()
  # suppresses an optional module's enable flag from appearing if we failed above check
  if(module_error)
    return()
  endif()

  if(module_OPTIONAL)
    option(ENABLE_MODULE_${name} "Enable optional module ${name}" OFF)
  else()
    # else this isn't an optional module, so turn it on!
    set(ENABLE_MODULE_${name} ON)
  endif()

  if(ENABLE_MODULE_${name})
    add_subdirectory("${directory}")
    maptk_create_doxygen("${name}" "${directory}")
    set(_maptk_modules_enabled
      ${_maptk_modules_enabled}
      ${name}
      CACHE INTERNAL "Active (turned on) maptk modules"
      )
  endif()
endfunction()
