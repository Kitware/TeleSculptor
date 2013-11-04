project( maptk )

include_directories("${maptk_all_SOURCE_DIR}")

add_subdirectory( core )

set( MAPTK_LIBRARIES
  "maptk_core"
  CACHE INTERNAL "" )

if( MAPTK_ENABLE_VXL )
  add_subdirectory( vxl )
  list(APPEND MAPTK_LIBRARIES "maptk_vxl")
endif()

include( ${CMAKE_SOURCE_DIR}/cmake/doxygen.cmake )

if( DOXYGEN_FOUND )
  set( DOCUMENTATION_OUTPUT_PATH
    ${CMAKE_BINARY_DIR}/doc )
#  create_doxygen(
#    ${DOCUMENTATION_OUTPUT_PATH}
#    ${CMAKE_CURRENT_SOURCE_DIR}/dir
#    module
#    ${DOCUMENTATION_OUTPUT_PATH}/othermodule.tag=../othermodule
#    ${DOCUMENTATION_OUTPUT_PATH}/othermodule2.tag=../othermodule2)
  create_doxygen(
    ${DOCUMENTATION_OUTPUT_PATH}
    ${CMAKE_CURRENT_SOURCE_DIR}/core
    core)
endif()
