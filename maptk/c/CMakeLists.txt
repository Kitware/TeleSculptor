# C interface library

option(MAPTK_ENABLE_C_LIB
  "Enable building of C interface library."
  OFF)

if( MAPTK_ENABLE_C_LIB )

  set(maptk_c_public_headers
    config.h
    config_block.h
    )
  set(maptk_c_private_headers
    c_utils.h
    )
  set(maptk_c_sources
    config_block.cxx
    )

  maptk_install_headers(
    ${maptk_c_public_headers}
    SUBDIR c
    )
  maptk_add_library( maptk_c
    ${maptk_c_public_headers}
    ${maptk_c_private_headers}
    ${maptk_c_sources}
    )
  target_link_libraries( maptk_c
    maptk #maptk_apm
    )

endif( MAPTK_ENABLE_C_LIB )
