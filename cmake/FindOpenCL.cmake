# - Try to find OpenCL
# This module tries to find an OpenCL implementation on your system. Currently
# it supports searching system locations or detecting environment variables
# for the following implementations:
#  AMD Advanced Parallel Processing SDK
#  NVIDIA CUDA Toolkit
#  Intel OpenCL SDK
#  Generic system installed version
#  Custom location
#
# To manually guide the search, define these environment or CMake variables:
#  OPENCL_ROOT          - Root path to search for include/CL/cl.h and
#                         lib{64}/{lib}OpenCL.{a,lib.so,dylib}
#  OPENCL_INCLUDE_ROOT  - Path to CL/cl.h, overrides OPENCL_ROOT
#  OPENCL_LIBRARY_ROOT  - Path to OpenCL library, overrides OPENCL_ROOT
#
#  Setting any of the _ROOT variables will override thier settings provided
#  by vendor specific environment variables
#
# Once done this will define
#  OPENCL_FOUND              - System has an OpenCL library
#  OPENCL_INCLUDE_DIRS       - The OpenCL include directories needed
#  OPENCL_LIBRARIES          - Link libraries needed for OpenCL
#  OPENCL_VERSION_STRING     - Version of OpenCL that was found
#  OPENCL_HAS_CXX            - Whether or not C++ bindings are available
#  OPENCL_CXX_VERSION_STRING - Version of the C++ bindings if available
#  OPENCL_CXX_DEFINITIONS    - Compiler defines needed for the C++ bindings
#                             (May be nexessary if C++ bindings are of a
#                              different version than the C API; i.e OpenCL 1.2
#                              but with C++ bindings for 1.1)
#

if(NOT OPENCL_FOUND)
  # Convert environment root variables into CMake variables
  if(NOT OPENCL_ROOT)
    set(OPENCL_ROOT $ENV{OPENCL_ROOT})
  endif()

  if(NOT OPENCL_INCLUDE_ROOT)
    set(OPENCL_INCLUDE_ROOT $ENV{OPENCL_INCLUDE_ROOT})
    if(NOT OPENCL_INCLUDE_ROOT AND OPENCL_ROOT)
      set(OPENCL_INCLUDE_ROOT ${OPENCL_ROOT}/include)
    endif()
  endif()

  if(NOT OPENCL_LIBRARY_ROOT)
    set(OPENCL_LIBRARY_ROOT $ENV{OPENCL_LIBRARY_ROOT})
    if(NOT OPENCL_LIBRARY_ROOT AND OPENCL_ROOT)
      get_property(_USE_LIB64 GLOBAL PROPERTY FIND_LIBRARY_USE_LIB64_PATHS)
      if(_USE_LIB64)
        set(OPENCL_LIBRARY_ROOT ${OPENCL_ROOT}/lib64)
      else()
        set(OPENCL_LIBRARY_ROOT ${OPENCL_ROOT}/lib)
      endif()
    endif()
  endif()

  # AMD APP SDK
  if(NOT "$ENV{AMDAPPSDKROOT}" STREQUAL "")
    set(OPENCL_ROOT $ENV{AMDAPPSDKROOT})
    if(NOT OPENCL_INCLUDE_ROOT)
      set(OPENCL_INCLUDE_ROOT ${OPENCL_ROOT}/include)
    endif()
    if(NOT OPENCL_LIBRARY_ROOT)
      include(CheckTypeSize)
      CHECK_TYPE_SIZE("void*" SIZEOF_VOID_P)
      if(SIZEOF_VOID_P EQUAL 4)
        set(OPENCL_LIBRARY_ROOT ${OPENCL_ROOT}/lib/x86)
      else()
        set(OPENCL_LIBRARY_ROOT ${OPENCL_ROOT}/lib/x86_64)
      endif()
    endif()

  # NVIDIA CUDA
  elseif(NOT "$ENV{CUDA_PATH}" STREQUAL "")
    message(STATUS "OpenCL: Searching for NVIDIA CUDA SDK")
    set(OPENCL_ROOT $ENV{CUDA_PATH})
    if(NOT OPENCL_INCLUDE_ROOT)
      set(OPENCL_INCLUDE_ROOT ${OPENCL_ROOT}/include)
    endif()
    if(NOT OPENCL_LIBRARY_ROOT)
      include(CheckTypeSize)
      CHECK_TYPE_SIZE("void*" SIZEOF_VOID_P)
      if(WIN32)
        if(SIZEOF_VOID_P EQUAL 4)
          set(OPENCL_LIBRARY_ROOT ${OPENCL_ROOT}/lib/Win32)
        else()
          set(OPENCL_LIBRARY_ROOT ${OPENCL_ROOT}/lib/x64)
        endif()
      else()
        if(SIZEOF_VOID_P EQUAL 4)
          set(OPENCL_LIBRARY_ROOT ${OPENCL_ROOT}/lib)
        else()
          set(OPENCL_LIBRARY_ROOT ${OPENCL_ROOT}/lib64)
        endif()
      endif()
    endif()

  # Intel OpenCL SDK
  elseif(NOT "$ENV{INTELOCLSDKROOT}" STREQUAL "")
    message(STATUS "OpenCL: Searching for Intel OpenCL SDK")
    set(OPENCL_ROOT $ENV{INTELOCLSDKROOT})
    if(NOT OPENCL_INCLUDE_ROOT)
      set(OPENCL_INCLUDE_ROOT ${OPENCL_ROOT}/include)
    endif()
    if(NOT OPENCL_LIBRARY_ROOT)
      include(CheckTypeSize)
      CHECK_TYPE_SIZE("void*" SIZEOF_VOID_P)
      if(WIN32)
        if(SIZEOF_VOID_P EQUAL 4)
          set(OPENCL_LIBRARY_ROOT ${OPENCL_ROOT}/lib/x86)
        else()
          set(OPENCL_LIBRARY_ROOT ${OPENCL_ROOT}/lib/x64)
        endif()
      else()
        if(SIZEOF_VOID_P EQUAL 4)
          set(OPENCL_LIBRARY_ROOT ${OPENCL_ROOT}/lib)
        else()
          set(OPENCL_LIBRARY_ROOT ${OPENCL_ROOT}/lib64)
        endif()
      endif()
    endif()

  endif()

  if(APPLE)
    set(_OPENCL_INCLUDE_BASE OpenCL)
  else()
    set(_OPENCL_INCLUDE_BASE CL)
  endif()

  # Find the headers
  if(OPENCL_INCLUDE_ROOT)
    set(_OPENCL_INCLUDE_OPTS PATHS ${OPENCL_INCLUDE_ROOT} NO_DEFAULT_PATH)
  endif()
  find_path(OPENCL_INCLUDE_DIR ${_OPENCL_INCLUDE_BASE}/cl.h
    ${_OPENCL_INCLUDE_OPTS}
  )
  if(OPENCL_INCLUDE_DIR)
    # Interrogate the C header for version information
    include(CheckSymbolExists)
    set(CMAKE_REQUIRED_INCLUDES ${OPENCL_INCLUDE_DIR})
    foreach(_MINOR_VER 0 1 2 3 4 5 6 7 8 9)
      CHECK_SYMBOL_EXISTS(CL_VERSION_1_${_MINOR_VER} "${_OPENCL_INCLUDE_BASE}/cl.h" _OPENCL_VER)
      if(_OPENCL_VER)
        set(OPENCL_VERSION_STRING "1.${_MINOR_VER}")
        unset(_OPENCL_VER CACHE)
      else()
        break()
      endif()
    endforeach()
    unset(CMAKE_REQUIRED_INCLUDES)

    if(EXISTS ${OPENCL_INCLUDE_DIR}/${_OPENCL_INCLUDE_BASE}/cl.hpp)
      set(OPENCL_HAS_CXX TRUE)

      # Interrogate the C++ header for seperate version information
      file(STRINGS ${OPENCL_INCLUDE_DIR}/${_OPENCL_INCLUDE_BASE}/cl.hpp
        _OPENCL_VER REGEX "version 1\\.[0-9]"
      )
      string(REGEX MATCH "1\\.([0-9])" OPENCL_CXX_VERSION_STRING
        "${_OPENCL_VER}"
      )

      # Handle the special case when you might be using a v1.2 C++ header with
      # v1.1 C APIs
      set(_MINOR_VER ${CMAKE_MATCH_1})
      if(OPENCL_CXX_VERSION_STRING VERSION_LESS OPENCL_VERSION_STRING)
        set(OPENCL_CXX_DEFINITIONS -DCL_USE_DEPRECATED_OPENCL_1_${_MINOR_VER}_APIS)
      endif()
    else()
      set(OPENCL_HAS_CXX FALSE)
    endif()
  endif()

  # Find the library
  if(OPENCL_LIBRARY_ROOT)
    set(_OPENCL_LIBRARY_OPTS PATHS ${OPENCL_LIBRARY_ROOT} NO_DEFAULT_PATH)
  endif()
  find_library(OPENCL_LIBRARY OpenCL
    ${_OPENCL_LIBRARY_OPTS}
  )

  include(FindPackageHandleStandardArgs)
  FIND_PACKAGE_HANDLE_STANDARD_ARGS(OpenCL
    REQUIRED_VARS OPENCL_INCLUDE_DIR OPENCL_LIBRARY
    VERSION_VAR OPENCL_VERSION_STRING
  )
  if(OPENCL_FOUND)
    set(OPENCL_INCLUDE_DIRS ${OPENCL_INCLUDE_DIR})
    set(OPENCL_LIBRARIES ${OPENCL_LIBRARY})
  endif()
endif()
