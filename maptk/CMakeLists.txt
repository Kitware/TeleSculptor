include(CheckCXXCompilerFlag)
check_cxx_compiler_flag(-fvisibility=hidden MAPTK_HAVE_GCC_VISIBILITY)

# Configure config.h.in
maptk_configure_file(config.h
  "${CMAKE_CURRENT_SOURCE_DIR}/config.h.in"
  "${CMAKE_CURRENT_BINARY_DIR}/config.h"
  MAPTK_HAVE_GCC_VISIBILITY
  )
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/config.h"
  DESTINATION include/maptk
  )

include_directories("${MAPTK_SOURCE_DIR}")

# Adding module subdirectories
maptk_add_module(core "${CMAKE_CURRENT_LIST_DIR}/core")

if(MAPTK_ENABLE_VXL)
  maptk_add_module(vxl "${CMAKE_CURRENT_LIST_DIR}/vxl")
endif()
