#
# Compiler flags specific to MSVC
#

option(MAPTK_ENABLE_DLL_WARNINGS "Enable warnings about DLL visibility." OFF)
if (NOT MAPTK_ENABLE_DLL_WARNINGS)
  maptk_check_compiler_flag(/wd4251)
  maptk_check_compiler_flag(/wd4275)
endif()

maptk_check_compiler_flag(/W3)

# Disable deprication warnings for standard C and STL functions in VS2005 and
# later.
if (MSVC_VERSION GREATER 1400 OR
    MSVC_VERSION EQUAL 1400)
  add_definitions(-D_CRT_NONSTDC_NO_DEPRECATE)
  add_definitions(-D_CRT_SECURE_NO_WARNINGS)
  add_definitions(-D_SCL_SECURE_NO_DEPRECATE)
endif()

# Prevent namespace pollution
add_definitions(-DWIN32_LEAN_AND_MEAN)
add_definitions(-DNOMINMAX)
