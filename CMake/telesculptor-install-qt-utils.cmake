# Useful CMake macros and functions

#Install the given plugins to the given destination
FUNCTION(INSTALL_PLUGINS plugins dest res)
  FOREACH(name ${plugins})
    FOREACH(conf Release Debug)
      STRING(TOUPPER ${conf} conf_case)
      GET_TARGET_PROPERTY(plugin_path_${conf} "${name}" LOCATION_${conf_case})
      IF(EXISTS "${plugin_path_${conf}}")
        GET_FILENAME_COMPONENT(plugin_file "${plugin_path_${conf}}" NAME)
        GET_FILENAME_COMPONENT(plugin_type "${plugin_path_${conf}}" PATH)
        GET_FILENAME_COMPONENT(plugin_type "${plugin_type}" NAME)
        SET(plugin_dst "${dest}/${plugin_type}")
        INSTALL(
          FILES "${plugin_path_${conf}}"
          CONFIGURATIONS "${conf}"
          DESTINATION "${plugin_dst}"
    )
        SET(tmp_res_${conf}
          "${tmp_res_${conf}};\$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}/${plugin_dst}/${plugin_file}")
        if (${conf} STREQUAL "Release")
          MESSAGE(STATUS "\tPlugin `${name}` will be installed.")
        endif()
      ELSE()
        MESSAGE(WARNING "Qt plugin install error: `${name}` not found.")
      ENDIF()
      SET(${res}_${conf} ${tmp_res_${conf}} PARENT_SCOPE)
    ENDFOREACH()
  ENDFOREACH()
ENDFUNCTION()

# Install plugins for specific qt modules at the given destination
FUNCTION(INSTALL_QT_PLUGINS modules dest res)
  FOREACH (qt_module ${${modules}})
    MESSAGE(STATUS "Parsing plugins for module Qt5::${qt_module}")
    SET(plugins ${Qt5${qt_module}_PLUGINS})
    INSTALL_PLUGINS(
      "${plugins}"
      "${dest}"
      inst_res
    )
    FOREACH(conf Release Debug)
      SET(tmp_res_${conf} "${tmp_res_${conf}};${inst_res_${conf}}")
    ENDFOREACH()
  ENDFOREACH()
  FOREACH(conf Release Debug)
    SET(${res}_${conf} ${tmp_res_${conf}} PARENT_SCOPE)
  ENDFOREACH()
ENDFUNCTION()
