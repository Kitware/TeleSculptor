find_package(Qt4 4.8 REQUIRED)
include(${QT_USE_FILE})

find_package(qtExtensions REQUIRED)
include(${qtExtensions_USE_FILE})

find_package(VTK 6.2 REQUIRED)

include(${VTK_USE_FILE})

#
# This option should be set if you want to do a bundle fixup for
# packaging.  The fixup is actually applied during install, but due to
# unexpected behaviour, all libraries are installed in the .../bin
# directory. This is okay for Windows and Mac, but unexpected for Linux.
#
option( MAPTK_FIXUP_BUNDLE_ON_PACKAGE "Perform bundle fixup when building a package. Only set when packaging." OFF )

include_directories(
  ${CMAKE_CURRENT_BINARY_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${MAPTK_SOURCE_DIR}
  ${MAPTK_BINARY_DIR}
)

if(vtkWebGLExporter_LOADED)
  message(STATUS "Module vtkWebGLExporter loaded, wekGL export enabled")
  add_definitions(-DVTKWEBGLEXPORTER)
endif()


set(gui_am_ui
  CameraView.ui
  MainWindow.ui
  MatchMatrixWindow.ui
  WorldView.ui
)

set(gui_ui
  ${gui_am_ui}
  AboutDialog.ui
  CameraOptions.ui
  DataColorOptions.ui
  DataFilterOptions.ui
  FeatureOptions.ui
  ImageOptions.ui
  PointOptions.ui
)

set(gui_resources
  icons/icons.qrc
  "${CMAKE_CURRENT_BINARY_DIR}/appdata.qrc"
)

set(gui_moc_headers
  AboutDialog.h
  ActorColorButton.h
  CameraOptions.h
  CameraView.h
  DataColorOptions.h
  DataFilterOptions.h
  FeatureOptions.h
  GradientSelector.h
  ImageOptions.h
  MainWindow.h
  MatchMatrixWindow.h
  PointOptions.h
  WorldView.h
  tools/AbstractTool.h
  tools/BundleAdjustTool.h
  tools/CanonicalTransformTool.h
  tools/NeckerReversalTool.h
)

set(gui_sources
  AboutDialog.cxx
  ActorColorButton.cxx
  CameraOptions.cxx
  CameraView.cxx
  DataColorOptions.cxx
  DataFilterOptions.cxx
  FeatureOptions.cxx
  GradientSelector.cxx
  ImageOptions.cxx
  MainWindow.cxx
  MatchMatrixAlgorithms.cxx
  MatchMatrixWindow.cxx
  PointOptions.cxx
  Project.cxx
  WorldView.cxx
  main.cxx
  vtkMaptkCamera.cxx
  vtkMaptkCameraRepresentation.cxx
  vtkMaptkFeatureTrackRepresentation.cxx
  vtkMaptkScalarsToGradient.cxx
  tools/AbstractTool.cxx
  tools/BundleAdjustTool.cxx
  tools/CanonicalTransformTool.cxx
  tools/NeckerReversalTool.cxx
)

maptk_use_appdata(
  ACKNOWLEDGMENTS
  BUILDINFO
  LICENSE
)

configure_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/appdata.qrc.in"
  "${CMAKE_CURRENT_BINARY_DIR}/appdata.qrc"
)

qt4_wrap_ui(gui_ui_sources ${gui_ui})
qt4_wrap_cpp(gui_moc_sources ${gui_moc_headers})
qt4_add_resources(gui_res_sources ${gui_resources})

qte_amc_wrap_ui(gui_amc_sources ActionManagerDialog ${gui_am_ui})

if (WIN32)
  set(gui_icon_source mapgui.rc)
endif()
if(APPLE)
  set(gui_icon_source icons/mapgui.icns)
  set(MACOSX_BUNDLE_ICON_FILE mapgui.icns)
  set_source_files_properties(icons/mapgui.icns PROPERTIES
    MACOSX_PACKAGE_LOCATION Resources)
endif()

set(no_install ON)
kwiver_add_executable(mapgui
  WIN32 MACOSX_BUNDLE
  ${gui_sources}
  ${gui_ui_sources}
  ${gui_moc_sources}
  ${gui_res_sources}
  ${gui_amc_sources}
  ${gui_icon_source}
)
unset(no_install)


target_link_libraries(mapgui
  maptk
  vital_apm
  vtkGUISupportQt
  vtkInteractionStyle
  vtkViewsCore
  vtkRenderingFreeTypeOpenGL
  vtkRenderingVolumeOpenGL
  vtkRenderingOpenGL
  vtkRenderingCore
  vtkCommonCore
  vtksys
  qtExtensions
  ${QT_QTMAIN_LIBRARY}
  ${QT_LIBRARIES}
)

if(vtkWebGLExporter_LOADED)
  target_link_libraries(mapgui vtkWebGLExporter)
endif()

if(APPLE)
  file(STRINGS "${MAPTK_SOURCE_DIR}/LICENSE" copyright_line
    LIMIT_COUNT 1 REGEX "Copyright")

  set_target_properties(mapgui PROPERTIES
    OUTPUT_NAME MAP-Tk
    MACOSX_BUNDLE_NAME "MAP-Tk"
    MACOSX_BUNDLE_INFO_STRING "MAP-Tk v${MAPTK_VERSION} GUI Application"
    MACOSX_BUNDLE_SHORT_VERSION_STRING "${MAPTK_VERSION}"
    MACOSX_BUNDLE_COPYRIGHT "${copyright_line}"
    )
endif()

kwiver_install(TARGETS mapgui
  ${exports}
  RUNTIME DESTINATION bin
  BUNDLE DESTINATION "${CMAKE_BUNDLE_LOCATION}")

# add DESTDIR switch (Turn it on if not WIN32?)
# do components bit (don't install header files)')
SET(plugin_dest_dir bin)
SET(qtconf_dest_dir bin)
set(dest_prefix "\${DESTDIR}\$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}")
set(gui-exe "${dest_prefix}/bin/mapgui${CMAKE_EXECUTABLE_SUFFIX}")
if(APPLE)
  set(gui-exe "${dest_prefix}/MacOS/MAP-Tk")
  set(plugin_dest_dir lib)
  set(qtconf_dest_dir Resources)
endif()

#
# Do bundle fixup
#    This could be conditional on APPLE || WIN32 also
#
if (MAPTK_FIXUP_BUNDLE_ON_PACKAGE)

  if (WIN32)
    # Add expected location of VTK dlls
    list(APPEND FIXUP_DIRS "${VTK_INSTALL_PREFIX}/bin")
  else()
    # Add expected location of VTK libraries
    list(APPEND FIXUP_DIRS "${VTK_INSTALL_PREFIX}/lib")
  endif()

  # For now, we're assuming we're pulling qtextensions from a build directory
  # (not an install).  Also, Mac and/or Linux will need be a bit different,
  # but will be handled later (they'd likely use CMAKE_BUILD_TYPE instead
  # of CMAKE_CONFIGURATION_TYPES). For Windows really expect the dll will
  # be in Release subdirectory, and so look there unless
  # CMAKE_CONFIGURATION_TYPES indicates only "debug"
  if(CMAKE_CONFIGURATION_TYPES)
    string(TOLOWER "${CMAKE_CONFIGURATION_TYPES}" BUILD_TYPE)
    if(BUILD_TYPE STREQUAL "debug")
      set(BINARY_DIR "Debug")
    else()
      set(BINARY_DIR "Release")
    endif()
  endif()
  list(APPEND FIXUP_DIRS "${qtExtensions_DIR}/${BINARY_DIR}")

  # Location for QT libraries
  list(APPEND FIXUP_DIRS "${QT_BIN_DIR}")
  list(APPEND FIXUP_DIRS "${QT_LIBRARY_DIR}")
  list(APPEND FIXUP_DIRS "${QT_BINARY_DIR}")

  get_property( maptk_plugins GLOBAL PROPERTY maptk_plugin_libraries)
  foreach( tgt ${maptk_plugins} )
    list(APPEND libs_genex "${dest_prefix}/lib/maptk/$<TARGET_FILE_NAME:${tgt}>")
  endforeach()

  get_property( bundle_path GLOBAL PROPERTY maptk_bundle_paths)

  list(APPEND FIXUP_DIRS "${VTK_CMAKE_DIR}../..")

  install(DIRECTORY "${QT_PLUGINS_DIR}/imageformats" DESTINATION ${plugin_dest_dir}/plugins COMPONENT Runtime)
  # install a qt.conf file
  install(CODE "
    file(WRITE \"${dest_prefix}/${qtconf_dest_dir}/qt.conf\" \"\")
    " COMPONENT Runtime)

  set( script_file  ${MAPTK_BINARY_DIR}/CMakeFiles/plugin-libs-$<CONFIGURATION>.cmake)

  file(GENERATE OUTPUT ${script_file}
    CONTENT "
set(plugin_libs \"${libs_genex}\")
set(bundle_path \"${bundle_path}\")
          ")

  install(CODE "
        file(GLOB_RECURSE QTPLUGINS
            \"${dest_prefix}/${plugin_dest_dir}/plugins/*${CMAKE_SHARED_LIBRARY_SUFFIX}\")
        set(BU_CHMOD_BUNDLE_ITEMS ON)
        set(ENV{VS_UNICODE_OUTPUT} \"\")
        include(${MAPTK_SOURCE_DIR}/CMake/utils/maptk-utils-bundle.cmake)
        include( ${MAPTK_BINARY_DIR}/CMakeFiles/plugin-libs-\${CMAKE_INSTALL_CONFIG_NAME}.cmake )
        list(APPEND FIXUP_DIRS ${FIXUP_DIRS} \${bundle_path})
        list(REMOVE_DUPLICATES FIXUP_DIRS)
        set(libs \${plugin_libs} \${QTPLUGINS})
        include(BundleUtilities)
        fixup_bundle(\"${gui-exe}\" \"\${libs}\" \"\${FIXUP_DIRS}\")
        ")

  if (WIN32)
    include(InstallRequiredSystemLibraries)
  endif()

endif(MAPTK_FIXUP_BUNDLE_ON_PACKAGE)
