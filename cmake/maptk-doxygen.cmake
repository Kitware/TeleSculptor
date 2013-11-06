#
# Setup and define MapTK Doxygen support
#

find_package(Doxygen)

if(DOXYGEN_FOUND)
  option(MAPTK_BUILD_DOCS "Build MapTK documentation via Doxygen with the ALL target." OFF)
  if(MAPTK_BUILD_DOCS)
    add_custom_target(doxygen ALL)
  else()
    add_custom_target(doxygen)
  endif()
endif()


#
# Register a directory to have Doxygen generate documentation over
#
# If Doxygen was not found, this method does nothing as documentation cannot
# be built, period. If Doxygen was found, but the documentation build option
# was turned off, the targets are still made, but are not added to the ALL
# build target.
#
# Arguments:
#   outputdir - Root directory to output generated documentation to.
#   inputdir  - Root input directory to configure Doxygen to cover.
#   name      - Name of this directory tree. This name creates a separate
#               section under the given ``outputdir``.
#
macro (maptk_create_doxygen outputdir inputdir name)
  if(DOXYGEN_FOUND)
    foreach (tag ${ARGN})
        string(REGEX REPLACE "=.*" "" tagfile ${tag})
        set(tagdeps
            ${tagdeps}
            ${tagfile})
    endforeach (tag)

    add_custom_command(
        OUTPUT  ${outputdir}/${name}
        COMMAND cmake -E make_directory ${outputdir}/${name}
        COMMENT "Creating documentation directory for ${name}")
    # Calling cmake script to configure and copy the Doxyfile template for
    # this specific directory.
    add_custom_command(
        OUTPUT  ${outputdir}/${name}/Doxyfile
        COMMAND ${CMAKE_COMMAND}
                -D "DOXYGEN_TEMPLATE=${CMAKE_SOURCE_DIR}/conf/Doxyfile.in"
                -D "DOXY_PROJECT_SOURCE_DIR=${inputdir}"
                -D "DOXY_DOCUMENTATION_OUTPUT_PATH=${outputdir}"
                -D "DOXY_PROJECT_NAME=${name}"
                -D "DOXY_TAG_FILES=${ARGN}"
                -P "${CMAKE_SOURCE_DIR}/cmake/maptk-doxygen-configure.cmake"
        DEPENDS ${CMAKE_SOURCE_DIR}/conf/Doxyfile.in
                ${outputdir}/${name}
        WORKING_DIRECTORY
                ${outputdir}/${name}
        COMMENT "Generating Doxyfile for ${name}")
    add_custom_command(
        OUTPUT  ${outputdir}/${name}.tag
        COMMAND ${DOXYGEN_EXECUTABLE}
        DEPENDS ${outputdir}/${name}/Doxyfile
        WORKING_DIRECTORY
                ${outputdir}/${name}
        COMMENT "Creating tag for ${name}")
    add_custom_command(
        OUTPUT  ${outputdir}/${name}/index.html
        COMMAND ${DOXYGEN_EXECUTABLE}
        DEPENDS ${outputdir}/${name}.tag
                ${tagdeps}
        WORKING_DIRECTORY
                ${outputdir}/${name}
        COMMENT "Creating HTML documentation for ${name}")
    add_custom_target(doxygen-${name}
        DEPENDS ${outputdir}/${name}/index.html)
    add_dependencies(doxygen
        doxygen-${name})
  endif()

endmacro (maptk_create_doxygen)
