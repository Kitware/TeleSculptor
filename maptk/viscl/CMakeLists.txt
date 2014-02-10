# maptk/viscl

find_package(OpenCL REQUIRED)
if(NOT OPENCL_HAS_CXX)
  message(WARNING "C++ bindings not found.  Using Kronos supplied reference OpenCL v1.1 C++ bindings.")
  list(APPEND OPENCL_INCLUDE_DIRS ${viscl_SOURCE_DIR}/kronos)
endif()
include_directories(SYSTEM ${OPENCL_INCLUDE_DIRS})
add_definitions(${OPENCL_CXX_DEFINITIONS})

set(maptk_viscl_headers_public
  descriptor_set.h
  detect_features.h
  extract_descriptors.h
  feature_set.h
  image_container.h
  match_features.h
  match_set.h
  viscl_config.h
  register.h
  )

set(maptk_viscl_sources
  descriptor_set.cxx
  detect_features.cxx
  extract_descriptors.cxx
  feature_set.cxx
  image_container.cxx
  match_features.cxx
  match_set.cxx
  register.cxx
  )

# installing header files
maptk_install_headers(
  ${maptk_viscl_headers_public}
  SUBDIR viscl
  )

maptk_add_library(maptk_viscl
  ${maptk_viscl_headers_public}
  ${maptk_viscl_sources}
  )

target_link_libraries( maptk_viscl maptk_core viscl_core viscl_tasks )
