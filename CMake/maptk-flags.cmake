#
# Encapsulation of flags that need to be set for MAPTK under different
# circumstances.
#

include(CheckCXXCompilerFlag)

# Infrastructure for conveniently adding flags
define_property(GLOBAL PROPERTY maptk_warnings
  BRIEF_DOCS "Warning flags for MAPTK build"
  FULL_DOCS "List of warning flags MAPTK will build with."
  )

# Helper function for adding compiler flags
function(maptk_check_compiler_flag flag)
  string(REPLACE "+" "plus" safeflag "${flag}")
  check_cxx_compiler_flag("${flag}" "has_compiler_flag-${safeflag}")
  if ("has_compiler_flag-${safeflag}")
    set_property(GLOBAL APPEND PROPERTY maptk_warnings "${flag}")
  endif()
endfunction ()

# Check for platform/compiler/IDE specific flags
if (MSVC)
  include(maptk-flags-msvc)
else()
  include(maptk-flags-gnu)
endif()

# Adding set flags via convenience structure to appropriate CMake property
get_property(maptk_cxx_flags GLOBAL PROPERTY maptk_warnings)
string(REPLACE ";" " " maptk_cxx_flags "${maptk_cxx_flags}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${maptk_cxx_flags}")
