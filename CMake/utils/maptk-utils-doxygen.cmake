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
#   maptk_create_doxygen(name input_dir [tagdep1 [tagdep2 ...]])
#
# Create documentation via Doxygen over the given inputdir. The name given is
# used to create the build targets. 'tagdep' arguments should be names of
# other documentation sets (i.e. module names) this set depends on.
#
# If Doxygen was not found, this method does nothing as documentation cannot
# be built, period.
#-
function(maptk_create_doxygen name inputdir)
  if(MAPTK_ENABLE_DOCS)
    message(STATUS "[doxy-${name}] Creating doxygen targets")

    # Constants -- could be moved outside this function?
    set(doxy_include_path       "${MAPTK_SOURCE_DIR};${MAPTK_BINARY_DIR}")
    set(doxy_doc_output_path    "${MAPTK_BINARY_DIR}/doc")
    set(doxy_files_dir "${MAPTK_SOURCE_DIR}/CMake/templates/doxygen")

    # current project specific variables
    set(doxy_project_name       "${name}")
    set(doxy_project_source_dir "${inputdir}")
    set(doxy_project_output_dir "${doxy_doc_output_path}/${doxy_project_name}")
    set(doxy_project_tag_file   "${doxy_project_output_dir}/${name}.tag")

    # Build up tag file and target dependency lists
    set(doxy_tag_files)
    set(tag_target_deps)
    message(STATUS "[doxy-${name}] given tag deps: \"${ARGN}\"")
    foreach (tag IN LISTS ARGN)
      message(STATUS "[doxy-${name}] - tag: ${tag}")
      list(APPEND doxy_tag_files
        "${doxy_doc_output_path}/${tag}/${tag}.tag=${doxy_doc_output_path}/${tag}"
        )
      list(APPEND tag_target_deps
        # Make creating a tag for a docset depend on the completion of the
        # entirety of its dependency docset, not just its tags (causes some
        # race conditions if just tags).
        doxygen-${tag}
        )
    endforeach (tag)
    string(REPLACE ";" " " doxy_tag_files "${doxy_tag_files}")
    message(STATUS "[doxy-${name}] tag files: '${doxy_tag_files}'")
    message(STATUS "[doxy-${name}] tag deps : '${tag_target_deps}'")

    # Make sure the output directory exists
    message(STATUS "[doxy-${name}] Creating directory creation command/target")
    add_custom_command(
      OUTPUT "${doxy_project_output_dir}"
      COMMAND ${CMAKE_COMMAND} -E make_directory "${doxy_project_output_dir}"
      COMMENT "Create documentation directory for ${name}"
      )

    # Configuring template files and linking known target names
    # Make sure targets get made, else this can't connect the dependency chain
    #set(no_configure_target TRUE)
    message(STATUS "[doxy-${name}] Configuring Doxyfile.common")
    maptk_configure_file(${name}-doxyfile.common
      "${doxy_files_dir}/Doxyfile.common.in"
      "${doxy_project_output_dir}/Doxyfile.common"
      doxy_project_name
      doxy_doc_output_path
      doxy_project_source_dir
      doxy_exclude_patterns
      doxy_include_path
      doxy_tag_files
      doxy_project_tag_file
      DEPENDS "${doxy_project_output_dir}"
      )
    message(STATUS "[doxy-${name}] Configuring Doxyfile.tag")
    maptk_configure_file(${name}-doxyfile.tag
      "${doxy_files_dir}/Doxyfile.tag.in"
      "${doxy_project_output_dir}/Doxyfile.tag"
      doxy_project_tag_file
      DEPENDS "${doxy_project_output_dir}"
      )
    message(STATUS "[doxy-${name}] Configuring Doxyfile")
    maptk_configure_file(${name}-doxyfile
      "${doxy_files_dir}/Doxyfile.in"
      "${doxy_project_output_dir}/Doxyfile"
      doxy_project_output_dir
      doxy_project_name
      DEPENDS "${doxy_project_output_dir}"
      )

    # Doxygen generation targets
    message(STATUS "[doxy-${name}] Creating tag generation target")
    add_custom_command(
      OUTPUT  "${doxy_project_tag_file}"
      COMMAND "${DOXYGEN_EXECUTABLE}" "${doxy_project_output_dir}/Doxyfile.tag"
      DEPENDS "${doxy_project_output_dir}/Doxyfile.common"
              "${doxy_project_output_dir}/Doxyfile.tag"
              ${tag_target_deps}
      WORKING_DIRECTORY
              "${doxy_project_output_dir}"
      COMMENT "Creating tag file for ${name}"
      )

    message(STATUS "[doxy-${name}] Creating doxygen generation target")
    add_custom_command(
      OUTPUT  "${doxy_project_output_dir}/index.html"
      COMMAND "${DOXYGEN_EXECUTABLE}" "${doxy_project_output_dir}/Doxyfile"
      DEPENDS "${doxy_project_output_dir}/Doxyfile.common"
              "${doxy_project_output_dir}/Doxyfile"
              "${doxy_project_tag_file}"
      WORKING_DIRECTORY
              "${doxy_project_output_dir}"
      COMMENT "Creating documentation pages for ${name}"
      )
    # top-level target doxygen project generation
    add_custom_target(doxygen-${name}
      DEPENDS "${doxy_project_output_dir}/index.html"
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
