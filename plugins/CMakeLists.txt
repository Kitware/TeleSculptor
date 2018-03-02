# Install the README
kwiver_install(
  FILES README.rst
  COMPONENT runtime
  DESTINATION share/maptk/${MAPTK_VERSION}/plugins
  )

# Install blender files
kwiver_install(
  DIRECTORY blender
  COMPONENT runtime
  DESTINATION share/maptk/${MAPTK_VERSION}/plugins
  )

# Install the zipped ruby file
kwiver_install(
  DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/sketchup"
  COMPONENT runtime
  DESTINATION share/maptk/${MAPTK_VERSION}/plugins
  )

# Move files to out of ./sketchup into the binary directory
add_custom_target(sketchup-movefiles
  COMMAND ${CMAKE_COMMAND} -E copy_directory
    ${CMAKE_CURRENT_SOURCE_DIR}/sketchup/kw_maptk ${CMAKE_CURRENT_BINARY_DIR}/kw_maptk
  COMMAND ${CMAKE_COMMAND} -E copy
    ${CMAKE_CURRENT_SOURCE_DIR}/sketchup/kw_maptk.rb ${CMAKE_CURRENT_BINARY_DIR}/kw_maptk.rb)

# Zip kw_maptk and kw_maptk.rb into kw_maptk.rbz
add_custom_target(sketchup-rubyzip
  COMMAND ${CMAKE_COMMAND} -E tar "cfx"
    "${CMAKE_CURRENT_BINARY_DIR}/kw_maptk.rbz" --format=zip
    "${CMAKE_CURRENT_BINARY_DIR}/kw_maptk.rb"
    "${CMAKE_CURRENT_BINARY_DIR}/kw_maptk")

# Put the *.rbz into ./sketchup
add_custom_target(sketchup-movezip ALL
  COMMAND ${CMAKE_COMMAND} -E make_directory
    "${CMAKE_CURRENT_BINARY_DIR}/sketchup"
  COMMAND ${CMAKE_COMMAND} -E copy
    "${CMAKE_CURRENT_BINARY_DIR}/kw_maptk.rbz"
    "${CMAKE_CURRENT_BINARY_DIR}/sketchup/kw_maptk.rbz")

# Enforce: files moved->files zipped->move zip
add_dependencies(sketchup-movezip sketchup-rubyzip)
add_dependencies(sketchup-rubyzip sketchup-movefiles)
