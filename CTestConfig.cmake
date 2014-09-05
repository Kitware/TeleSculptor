## This file should be placed in the root directory of your project.
## Then modify the CMakeLists.txt file in the root directory of your
## project to incorporate the testing dashboard.
## # The following are required to uses Dart and the Cdash dashboard
##   ENABLE_TESTING()
##   INCLUDE(CTest)
set(CTEST_PROJECT_NAME "MAPTK")
set(CTEST_NIGHTLY_START_TIME "01:00:00 UTC")

if (NOT MAPTK_INTERNAL_DASHBOARD)
  set(CTEST_DROP_METHOD "http")
  set(CTEST_DROP_SITE "open.cdash.org")
  set(CTEST_DROP_LOCATION "/submit.php?project=MAPTK")
else()
  set(CTEST_DROP_METHOD "https")
  set(CTEST_DROP_SITE "www.kitware.com/CDash")
  set(CTEST_DROP_LOCATION "/submit.php?project=maptk-private")
endif()

set(CTEST_DROP_SITE_CDASH TRUE)
