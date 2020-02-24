# Install the README
kwiver_install(
  FILES README.rst
  COMPONENT runtime
  DESTINATION share/telesculptor/${TeleSculptor_VERSION}/plugins
  )

# Install blender files
kwiver_install(
  DIRECTORY blender
  COMPONENT runtime
  DESTINATION share/telesculptor/${TeleSculptor_VERSION}/plugins
  )

# Install the zipped ruby file
kwiver_install(
  FILES "${CMAKE_CURRENT_BINARY_DIR}/kw_telesculptor.rbz"
  COMPONENT runtime
  DESTINATION share/telesculptor/${TeleSculptor_VERSION}/plugins/sketchup
  )

# Move files to out of ./sketchup into the binary directory
add_custom_target(sketchup-movefiles
  COMMAND ${CMAKE_COMMAND} -E copy_directory
    ${CMAKE_CURRENT_SOURCE_DIR}/sketchup/kw_telesculptor ${CMAKE_CURRENT_BINARY_DIR}/kw_telesculptor
  COMMAND ${CMAKE_COMMAND} -E copy
    ${CMAKE_CURRENT_SOURCE_DIR}/sketchup/kw_telesculptor.rb ${CMAKE_CURRENT_BINARY_DIR}/kw_telesculptor.rb)

# Zip kw_telesculptor and kw_telesculptor.rb into kw_telesculptor.rbz
add_custom_target(sketchup-rubyzip ALL
  COMMAND ${CMAKE_COMMAND} -E tar "cf"
    "${CMAKE_CURRENT_BINARY_DIR}/kw_telesculptor.rbz" --format=zip
    "${CMAKE_CURRENT_BINARY_DIR}/kw_telesculptor.rb"
    "${CMAKE_CURRENT_BINARY_DIR}/kw_telesculptor")

# Enforce: files moved->files zipped
add_dependencies(sketchup-rubyzip sketchup-movefiles)
