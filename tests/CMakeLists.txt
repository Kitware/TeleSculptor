project( maptk_tests )

set(no_install TRUE)

if (WIN32)
  # TODO: Output to a different directory and then use $<CONFIGURATION> in the
  # working path when generator expressions are supported in test properties.
  set(maptk_test_output_path
    "${maptk_output_dir}/bin")
else ()
  set(maptk_test_output_path
    "${maptk_binary_dir}/tests/bin")
  set(maptk_test_working_path
    "${maptk_binary_dir}/tests")
endif ()

include("${maptk_source_dir}/conf/maptk-macro-tests.cmake")

set(maptk_test_data_directory
  "${CMAKE_CURRENT_SOURCE_DIR}/data")


include_directories("${CMAKE_CURRENT_SOURCE_DIR}")
include_directories("${MAPTK_SOURCE_DIR}")
include_directories("${MAPTK_BINARY_DIR}")

add_subdirectory( core )

