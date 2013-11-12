#
# Setup and define MAPTK Doxygen support
#

find_package(Doxygen)

cmake_dependent_option(MAPTK_ENABLE_DOCS
  "Build MAPTK documentation via Doxygen." OFF
  DOXYGEN_FOUND OFF
  )
cmake_dependent_option(MAPTK_INSTALL_DOCS
  "Install built Doxygen documentation." OFF
  MAPTK_ENABLE_DOCS OFF
  )

if(MAPTK_ENABLE_DOCS)
  add_custom_target(doxygen ALL)
endif()


#+
# Register a directory to have Doxygen generate documentation.
#
#   maptk_create_doxygen(inputdir name [tagdep1 [tagdep2 ...]])
#
# Create documentation via Doxygen over the given inputdir. The name given is
# used to create the build targets. 'tagdep' arguments should be names of
# other documentation sets this set depends on.
#
# If Doxygen was not found, this method does nothing as documentation cannot
# be built, period.
#-
function(maptk_create_doxygen name inputdir)
  if(MAPTK_ENABLE_DOCS)
    message(STATUS "[doxy-${name}] Creating doxygen targets")

    set(doxy_project_name       "${name}")
    set(doxy_project_source_dir "${inputdir}")
    set(doxy_include_path     "${MAPTK_BINARY_DIR}/maptk")
    #set(doxy_include_path       "${MAPTK_SOURCE_DIR}")
    set(doxy_doc_output_path    "${MAPTK_BINARY_DIR}/doc")

    set(doxy_files_dir "${MAPTK_SOURCE_DIR}/cmake/templates/doxygen")

    # Build up tag file and target dependency lists
    set(doxy_tag_files)
    set(tag_target_deps)
    foreach (tag IN LISTS ${ARGN})
      list(APPEND doxy_tag_files
        "${doxy_doc_output_path}/${tag}.tag=../${tag}"
        )
      list(APPEND tag_target_deps
        doxygen-${tag}-tag
        )
    endforeach (tag)
    string(REPLACE ";" " " doxy_tag_files "${doxy_tag_files}")
    message(STATUS "[doxy-${name}] tag files: '${doxy_tag_files}'")
    message(STATUS "[doxy-${name}] tag deps : '${tag_target_deps}'")

    message(STATUS "[doxy-${name}] Creating directory creation target")
    add_custom_target(doxygen-${name}-dir
        COMMAND cmake -E make_directory "${doxy_doc_output_path}/${name}"
        COMMENT "Creating documentation directory for ${name}"
        )

    # Configuring template files and linking known target names
    # Make sure targets get made, else this can't connect the dependency chain
    set(no_configure_target FALSE)
    message(STATUS "[doxy-${name}] Configuring Doxyfile.common")
    maptk_configure_file(${name}-doxyfile.common
      "${doxy_files_dir}/Doxyfile.common.in"
      "${doxy_doc_output_path}/${name}/Doxyfile.common"
      doxy_project_name
      doxy_doc_output_path
      doxy_project_source_dir
      doxy_exclude_patterns
      doxy_include_path
      doxy_tag_files
      )
    message(STATUS "[doxy-${name}] Configuring Doxyfile.tag")
    maptk_configure_file(${name}-doxyfile.tag
      "${doxy_files_dir}/Doxyfile.tag.in"
      "${doxy_doc_output_path}/${name}/Doxyfile.tag"
      doxy_doc_output_path
      doxy_project_name
      )
    message(STATUS "[doxy-${name}] Configuring Doxyfile")
    maptk_configure_file(${name}-doxyfile
      "${doxy_files_dir}/Doxyfile.in"
      "${doxy_doc_output_path}/${name}/Doxyfile"
      doxy_doc_output_path
      doxy_project_name
      )
    message(STATUS "[doxy-${name}] Linking configuration depencencies")
    add_dependencies(configure-${name}-doxyfile.common
      doxygen-${name}-dir
      )
    add_dependencies(configure-${name}-doxyfile.tag
      doxygen-${name}-dir
      )
    add_dependencies(configure-${name}-doxyfile
      doxygen-${name}-dir
      )

    # Doxygen generated target
    message(STATUS "[doxy-${name}] Creating tag generation target")
    add_custom_target(doxygen-${name}-tag
      DEPENDS configure-${name}-doxyfile.common
              configure-${name}-doxyfile.tag
      COMMAND "${DOXYGEN_EXECUTABLE}"
              "${doxy_doc_output_path}/${name}/Doxyfile.tag"
      WORKING_DIRECTORY
              "${doxy_doc_output_path}/${name}"
      COMMENT "Creating tag for ${name}."
      )

    message(STATUS "[doxy-${name}] Creating doxygen generation target")
    add_custom_target(doxygen-${name}
      DEPENDS configure-${name}-doxyfile.common
              configure-${name}-doxyfile
              doxygen-${name}-tag
              ${tag_target_deps}
      COMMAND "${DOXYGEN_EXECUTABLE}"
              "${doxy_doc_output_path}/${name}/Doxyfile"
      WORKING_DIRECTORY
              "${doxy_doc_output_path}/${name}"
      COMMENT "Creating documentation for ${name}."
      )

    message(STATUS "[doxy-${name}] Linking to top-level doxygen target")
    add_dependencies(doxygen
        doxygen-${name}
        )

    if(MAPTK_INSTALL_DOCS)
      message(STATUS "[doxy-${name}] marking for install")
      maptk_install(
        DIRECTORY   "${doxy_doc_output_path}/${name}/"
        DESTINATION "share/doc/maptk-${MAPTK_VERSION}/${name}"
        COMPONENT   documentation
        )
    endif()
  endif()
endfunction(maptk_create_doxygen)
