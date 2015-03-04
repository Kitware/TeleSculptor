# MAPTK Python interface support

include(CMakeDependentOption)

cmake_dependent_option( MAPTK_ENABLE_PYTHON
  "Enable configuration and installation of MAPTK Python bindings based on C interface"
  ON                  # Default value
  MAPTK_ENABLE_C_LIB  # Dependency expression
  OFF                 # State when dependency expression is false
  )
cmake_dependent_option( MAPTK_PYTHON_SEARCH_BUILD_DIR
  "Include build directory in search paths when python looks for C interface library."
  ON
  "MAPTK_ENABLE_PYTHON;BUILD_SHARED_LIBS OR MAPTK_C_LIB_FORCE_SHARED"
  OFF
  )


if( MAPTK_ENABLE_PYTHON )
  if( BUILD_SHARED_LIBS OR MAPTK_C_LIB_FORCE_SHARED )
    if( MAPTK_PYTHON_SEARCH_BUILD_DIR )
      set(MAPTK_BUILD_DIRECTORY "${MAPTK_BINARY_DIR}")
    endif()

    maptk_configure_file(python-find_maptk_libraray.py
      "${CMAKE_CURRENT_SOURCE_DIR}/maptk/util/find_maptk_library.py.in"
      "${CMAKE_CURRENT_SOURCE_DIR}/maptk/util/find_maptk_library.py"
      MAPTK_BUILD_DIRECTORY
      )

    #TODO: Install rules for python module and files
  else()
    message(WARNING
      "Halting python Python bindings processing due to not building shared "
      "libraries. Python cannot load static libraries."
      )
  endif()
endif( MAPTK_ENABLE_PYTHON )
