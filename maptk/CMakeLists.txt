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

if(MAPTK_ENABLE_OPENCV)
  maptk_add_module(ocv "${CMAKE_CURRENT_LIST_DIR}/ocv")
endif()

if(MAPTK_ENABLE_PROJ)
  maptk_add_module(proj "${CMAKE_CURRENT_LIST_DIR}/proj")
endif()

# Configure a header file to indicate which modules are enabled
get_property(current_modules GLOBAL PROPERTY maptk_modules_enabled)
foreach(mod IN LISTS current_modules)
  string(TOUPPER "MAPTK_HAVE_MODULE_${mod}" mod_define)
  set(${mod_define} 1)
  list(APPEND module_defines ${mod_define})
endforeach()
maptk_configure_file(modules.h
  "${CMAKE_CURRENT_SOURCE_DIR}/modules.h.in"
  "${CMAKE_CURRENT_BINARY_DIR}/modules.h"
  ${module_defines}
  )
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/modules.h"
  DESTINATION include/maptk
  )
