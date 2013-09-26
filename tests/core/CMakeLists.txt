# Tests that don't require any arguments at runtime
#
set( no_argument_test_sources
  test_point_2d.cxx
)

# Tests that take the data directory as the only argument at runtime
#
set( data_argument_test_sources
  # None yet
)

create_test_sourcelist( test_sources
  test_driver.cxx

  # List of source files
  ${no_argument_test_sources}
  ${data_argument_test_sources}
)


add_executable( maptk_test_core
  ${test_sources}
)
target_link_libraries( maptk_test_core maptk_core )

# Create ADD_TEST lines for some tests automatically.  Tests taking
# special arguments will be specified manually later.

foreach( filename ${no_argument_test_sources} )
  string( REPLACE ".cxx" "" filebase ${filename} )
  string( REPLACE "test_" "" testname ${filebase} )
  add_test( NAME maptk_core_${testname}
            COMMAND $<TARGET_FILE:maptk_test_core> ${filebase} )
endforeach( filename )

foreach( filename ${data_argument_test_sources} )
  string( REPLACE ".cxx" "" filebase ${filename} )
  string( REPLACE "test_" "" testname ${filebase} )
  add_test( NAME maptk_core_${testname}
            COMMAND $<TARGET_FILE:maptk_test_core> ${filebase}
                    ${maptk_tests_SOURCE_DIR}/data ${CMAKE_CURRENT_BINARY_DIR} )
endforeach( filename )
