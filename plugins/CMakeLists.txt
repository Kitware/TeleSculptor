set(maptk_plugins
    blender
    sketchup
    )

# Install the README
kwiver_install(
  FILES README.rst
  COMPONENT runtime
  DESTINATION share/maptk/${MAPTK_VERSION}/plugins
  )

# Install plugin files
kwiver_install(
  DIRECTORY ${maptk_plugins}
  COMPONENT runtime
  DESTINATION share/maptk/${MAPTK_VERSION}/plugins
  )
