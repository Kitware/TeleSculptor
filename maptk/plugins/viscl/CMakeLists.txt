# Build / install plugin containing VisCL algorithm implementations + support
# structures

set( maptk_viscl_headers_public
  convert_image.h
  descriptor_set.h
  detect_features.h
  extract_descriptors.h
  feature_set.h
  image_container.h
  match_features.h
  match_set.h
  viscl_config.h
  utils.h
  )

set( maptk_viscl_sources
  algorithm_impl_pl_interface.cxx
  convert_image.cxx
  descriptor_set.cxx
  detect_features.cxx
  extract_descriptors.cxx
  feature_set.cxx
  image_container.cxx
  match_features.cxx
  match_set.cxx
  utils.cxx
  )

# installing header files
maptk_install_plugin_headers( viscl
  ${maptk_viscl_headers_public}
  )
maptk_add_plugin( viscl
  MAKE_PLUGIN_VISCL
  ${maptk_viscl_sources}
  )
maptk_plugin_link_libs( viscl
  viscl_core viscl_tasks viscl_vxl
  )
