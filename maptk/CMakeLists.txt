project( maptk )

add_subdirectory( core )


set( MAPTK_LIBRARIES
  "maptk_core"
  CACHE INTERNAL "" )

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
