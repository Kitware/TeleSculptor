# Install config files
kwiver_install(
  DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/"
  COMPONENT runtime
  DESTINATION share/telesculptor/${TeleSculptor_VERSION}/examples
  PATTERN "CMakeLists.txt" EXCLUDE
  )
