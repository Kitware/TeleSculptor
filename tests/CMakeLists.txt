project( maptk_tests )

set(no_install TRUE)

# ==================================================================
###
# Require Boost if std::random is not available
#

if (NOT VITAL_USE_STD_RANDOM)

  # Required Boost external dependency
  if(WIN32)
    set(Boost_USE_STATIC_LIBS TRUE)
  endif()

  find_package(Boost 1.50)
  add_definitions(-DBOOST_ALL_NO_LIB)
  include_directories(SYSTEM ${Boost_INCLUDE_DIRS})

endif()

if (WIN32)
  # TODO: Output to a different directory and then use $<CONFIGURATION> in the
  # working path when generator expressions are supported in test properties.
  set(kwiver_test_output_path
    "${MAPTK_BINARY_DIR}/bin")
else ()
  set(kwiver_test_output_path
    "${MAPTK_BINARY_DIR}/tests/bin")
  set(kwiver_test_working_path
    "${MAPTK_BINARY_DIR}/tests")
endif ()


set(maptk_test_data_directory
  "${CMAKE_CURRENT_SOURCE_DIR}/data")


include_directories("${CMAKE_CURRENT_SOURCE_DIR}")
include_directories("${MAPTK_SOURCE_DIR}")
include_directories("${MAPTK_BINARY_DIR}")

add_subdirectory(maptk)
