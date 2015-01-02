#
# Compiler flags specific for use with Clang
#
include( maptk-flags-unix_common )

option(MAPTK_CLANG_WEVERYTHING OFF
  "Enable use of Weverything compiler flag, which outputs literally every warning possible."
  )
if( MAPTK_CLANG_WEVERYTHING )
  maptk_check_compiler_flag(-Weverything)
endif()
