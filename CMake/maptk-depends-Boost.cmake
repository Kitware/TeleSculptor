# Required Boost external dependency

if(WIN32)
  set(Boost_USE_STATIC_LIBS TRUE)
endif()

find_package(Boost 1.50 REQUIRED COMPONENTS system timer chrono)
add_definitions(-DBOOST_ALL_NO_LIB)
include_directories(SYSTEM ${Boost_INCLUDE_DIRS})
