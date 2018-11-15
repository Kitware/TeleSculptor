set(CMAKE_AUTOMOC ON)
set(Qt5_MODULES Core Designer UiPlugin Widgets Xml)
find_package(Qt5 5.7 COMPONENTS ${Qt5_MODULES} REQUIRED)
set(QT_LIBRARIES )
foreach(module ${Qt5_MODULES})
  list(APPEND QT_LIBRARIES Qt5::${module})
endforeach()

find_package(qtExtensions REQUIRED)
include(${qtExtensions_USE_FILE})
if(QTE_QT_VERSION VERSION_EQUAL "4")
  message(FATAL_ERROR "${PROJECT_NAME} does not support Qt4. "
    "But QTE_QT_VERSION is ${QTE_QT_VERSION}. "
    "Please provide path to qtExtensions built with Qt version 5 or higher.")
endif()

find_package(VTK REQUIRED
  COMPONENTS
  vtkFiltersSources
  vtkGUISupportQt
  vtkIOImage
  vtkIOPLY
  vtkIOXML
  vtkImagingCore
  vtkInteractionStyle
  vtkInteractionWidgets
  vtkRenderingAnnotation
  vtkRenderingFreeType
  )
if(VTK_VERSION VERSION_LESS 8.1)
  message(FATAL_ERROR "${PROJECT_NAME} supports VTK >= v8.1 "
    "(Found ${VTK_VERSION})")
endif()
include(${VTK_USE_FILE})
if(VTK_QT_VERSION VERSION_EQUAL "4")
  message(FATAL_ERROR "${PROJECT_NAME} does not support Qt4. "
    "But VTK_QT_VERSION is ${VTK_QT_VERSION}. "
    "Please provide path to VTK built with Qt version 5 or higher.")
endif()
if(VTK_RENDERING_BACKEND STREQUAL "OpenGL")
  message(FATAL_ERROR "${PROJECT_NAME} does not support VTK's OpenGL backend.\n"
    "The old OpenGL backend is known to have rendering issues and "
    "has been deprecated in newer versions of VTK.\n"
    "Please provide path to VTK built with VTK_RENDERING_BACKEND = OpenGL2.")
endif()

#
# This option should be set if you want to do a bundle fixup for
# packaging.  The fixup is actually applied during install, but due to
# unexpected behaviour, all libraries are installed in the .../bin
# directory. This is okay for Windows and Mac, but unexpected for Linux.
#
option( MAPTK_FIXUP_BUNDLE_ON_PACKAGE
  "Perform bundle fixup when building a package. Only set when packaging."
  OFF )

include_directories(
  ${CMAKE_CURRENT_BINARY_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${MAPTK_SOURCE_DIR}
  ${MAPTK_BINARY_DIR}
)

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
  ColorizeSurfaceOptions.ui
  DataColorOptions.ui
  DataFilterOptions.ui
  DepthMapFilterOptions.ui
  DepthMapOptions.ui
  DepthMapView.ui
  DepthMapViewOptions.ui
  FeatureOptions.ui
  ImageOptions.ui
  PointOptions.ui
  VolumeOptions.ui
)

set(gui_resources
  icons/icons.qrc
  "${CMAKE_CURRENT_BINARY_DIR}/appdata.qrc"
)

set(gui_sources
  AboutDialog.cxx
  ActorColorButton.cxx
  CameraOptions.cxx
  CameraView.cxx
  ColorizeSurfaceOptions.cxx
  DataColorOptions.cxx
  DataFilterOptions.cxx
  DepthMapFilterOptions.cxx
  DepthMapOptions.cxx
  DepthMapView.cxx
  DepthMapViewOptions.cxx
  FeatureOptions.cxx
  GradientSelector.cxx
  GroundControlPointsHelper.cxx
  GroundControlPointsWidget.cxx
  GuiCommon.cxx
  ImageOptions.cxx
  MainWindow.cxx
  MatchMatrixAlgorithms.cxx
  MatchMatrixWindow.cxx
  MetadataView.cxx
  PointOptions.cxx
  Project.cxx
  VideoImport.cxx
  VolumeOptions.cxx
  WorldView.cxx
  main.cxx
  vtkMaptkCamera.cxx
  vtkMaptkCameraRepresentation.cxx
  vtkMaptkFeatureTrackRepresentation.cxx
  vtkMaptkImageDataGeometryFilter.cxx
  vtkMaptkImageUnprojectDepth.cxx
  vtkMaptkScalarDataFilter.cxx
  vtkMaptkScalarsToGradient.cxx
  vtkMaptkSeedWidget.cxx
  tools/AbstractTool.cxx
  tools/BundleAdjustTool.cxx
  tools/CanonicalTransformTool.cxx
  tools/ComputeDepthTool.cxx
  tools/InitCamerasLandmarksTool.cxx
  tools/MeshColoration.cxx
  tools/NeckerReversalTool.cxx
  tools/ReconstructionData.cxx
  tools/SaveFrameTool.cxx
  tools/SaveKeyFrameTool.cxx
  tools/TrackFeaturesSprokitTool.cxx
  tools/TrackFeaturesTool.cxx
  tools/TrackFilterTool.cxx
  tools/TriangulateTool.cxx
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

qt5_wrap_ui(gui_ui_sources ${gui_ui})
qt5_add_resources(gui_res_sources ${gui_resources})

qte_amc_wrap_ui(gui_amc_sources ActionManagerDialog ${gui_am_ui})

if (WIN32)
  set(gui_icon_source TeleSculptor.rc)
endif()
if(APPLE)
  set(gui_icon_source icons/TeleSculptor.icns)
  set(MACOSX_BUNDLE_ICON_FILE TeleSculptor.icns)
  set_source_files_properties(icons/TeleSculptor.icns PROPERTIES
    MACOSX_PACKAGE_LOCATION Resources)
endif()

set(no_install ON)
kwiver_add_executable(TeleSculptor
  WIN32 MACOSX_BUNDLE
  ${gui_sources}
  ${gui_ui_sources}
  ${gui_res_sources}
  ${gui_amc_sources}
  ${gui_icon_source}
)
unset(no_install)

target_link_libraries(TeleSculptor
  maptk
  kwiver::kwiver_adapter
  kwiver::kwiver_algo_core
  kwiver::sprokit_pipeline
  kwiver::vital_vpm
  qtExtensions
  ${VTK_LIBRARIES}
  ${QT_LIBRARIES}
)

if(vtkWebGLExporter_LOADED)
  message(STATUS "Module vtkWebGLExporter present: wekGL export enabled")
  target_compile_definitions(TeleSculptor PRIVATE -DVTKWEBGLEXPORTER)
  target_link_libraries(TeleSculptor vtkWebGLExporter)
endif()

if(APPLE)
  file(STRINGS "${MAPTK_SOURCE_DIR}/LICENSE" copyright_line
    LIMIT_COUNT 1 REGEX "Copyright")

  set_target_properties(TeleSculptor PROPERTIES
    OUTPUT_NAME TeleSculptor
    MACOSX_BUNDLE_NAME "MAP-Tk TeleSculptor"
    MACOSX_BUNDLE_INFO_STRING "MAP-Tk v${MAPTK_VERSION} TeleSculptor Application"
    MACOSX_BUNDLE_SHORT_VERSION_STRING "${MAPTK_VERSION}"
    MACOSX_BUNDLE_COPYRIGHT "${copyright_line}"
    )
endif()

kwiver_install(TARGETS TeleSculptor
  ${exports}
  RUNTIME DESTINATION bin
  BUNDLE DESTINATION "${CMAKE_BUNDLE_LOCATION}")

# add DESTDIR switch (Turn it on if not WIN32?)
# do components bit (don't install header files)')
SET(plugin_dest_dir bin)
SET(qtconf_dest_dir bin)
set(dest_prefix "\${DESTDIR}\$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}")
set(gui-exe "${dest_prefix}/bin/TeleSculptor${CMAKE_EXECUTABLE_SUFFIX}")
if(APPLE)
  set(gui-exe "${dest_prefix}/MacOS/TeleSculptor")
  set(plugin_dest_dir lib)
  set(qtconf_dest_dir Resources)
endif()

#
# Do bundle fixup
#    This could be conditional on APPLE || WIN32 also
#
if (MAPTK_FIXUP_BUNDLE_ON_PACKAGE)
  include(${MAPTK_SOURCE_DIR}/CMake/maptk-install-qt-utils.cmake)

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

  list(APPEND FIXUP_DIRS "${KWIVER_LIBRARY_DIR}/${BINARY_DIR}")
  # Windows also has libraries (DLLs) in the bin directory
  if(WIN32)
    list(APPEND FIXUP_DIRS "${KWIVER_DIR}/bin/${BINARY_DIR}")
  endif()

  if(DEFINED fletch_ROOT)
    if(WIN32)
      list(APPEND FIXUP_DIRS "${fletch_ROOT}/bin")
      # this is a hack for now.  Fletch should really export paths like this.
      list(APPEND FIXUP_DIRS "${fletch_ROOT}/x64/vc14/bin")
      list(APPEND FIXUP_DIRS "${fletch_ROOT}/x64/vc15/bin")
    else()
      list(APPEND FIXUP_DIRS "${fletch_ROOT}/lib")
    endif()
  endif()


  # Location for QT libraries
  if(WIN32 AND TARGET Qt5::Core)
    get_property(_Qt5_Core_LOCATION TARGET Qt5::Core PROPERTY LOCATION)
    get_filename_component(QT_BINARY_DIR "${_Qt5_Core_LOCATION}" PATH)
  endif()
  list(APPEND FIXUP_DIRS "${QT_BIN_DIR}")
  list(APPEND FIXUP_DIRS "${QT_LIBRARY_DIR}")
  list(APPEND FIXUP_DIRS "${QT_BINARY_DIR}")

  install(DIRECTORY "${KWIVER_DIR}/lib/${BINARY_DIR}/kwiver/modules" DESTINATION lib
          COMPONENT Runtime
          FILES_MATCHING PATTERN "*plugin${CMAKE_SHARED_MODULE_SUFFIX}" )

  install(DIRECTORY "${KWIVER_DIR}/lib/${BINARY_DIR}/kwiver/modules/"
          DESTINATION ${plugin_dest_dir}
          COMPONENT Runtime
          FILES_MATCHING PATTERN "*logger${CMAKE_SHARED_MODULE_SUFFIX}"
          PATTERN "plugin_explorer" EXCLUDE )

  install(DIRECTORY "${KWIVER_DIR}/lib/${BINARY_DIR}/kwiver/processes" DESTINATION lib
          COMPONENT Runtime
          FILES_MATCHING PATTERN "*${CMAKE_SHARED_MODULE_SUFFIX}" )

  install(DIRECTORY "${fletch_ROOT}/share/proj"
          DESTINATION share COMPONENT Runtime)

  set(kwiver_tools
      config_explorer
      dump_klv
      pipe_config
      pipe_to_dot
      pipeline_runner
      plugin_explorer)
  foreach(tool ${kwiver_tools})
    install(PROGRAMS "${KWIVER_DIR}/bin/${BINARY_DIR}/${tool}${CMAKE_EXECUTABLE_SUFFIX}"
            DESTINATION bin
            COMPONENT Runtime)
  endforeach()

  get_property( bundle_path GLOBAL PROPERTY maptk_bundle_paths)

  list(APPEND FIXUP_DIRS "${VTK_CMAKE_DIR}../..")

  # install(DIRECTORY "${QT_PLUGINS_DIR}/imageformats" DESTINATION ${plugin_dest_dir}/plugins COMPONENT Runtime)
  set(QT5_MODULES "Gui;Widgets")
  install_qt_plugins(QT5_MODULES "${plugin_dest_dir}/plugins" QT_PLUGINS)
  # install a qt.conf file
  install(CODE "
    file(WRITE \"${dest_prefix}/${qtconf_dest_dir}/qt.conf\" \"\")
    " COMPONENT Runtime)

  set( script_file  ${MAPTK_BINARY_DIR}/CMakeFiles/plugin-libs-$<CONFIGURATION>.cmake)

  file(GENERATE OUTPUT ${script_file}
    CONTENT "
set(bundle_path \"${bundle_path}\")
          ")

  install(CODE "
        set(QT_PLUGINS_Release \"${QT_PLUGINS_Release}\")
        set(QT_PLUGINS_Debug \"${QT_PLUGINS_Debug}\")
        string(CONCAT QTPLUGINS \"QT_PLUGINS_\" \"\${CMAKE_INSTALL_CONFIG_NAME}\")
        file(GLOB_RECURSE QTPLUGINS
            \"${dest_prefix}/${plugin_dest_dir}/plugins/*${CMAKE_SHARED_LIBRARY_SUFFIX}\")
        file(GLOB_RECURSE KWIVER_PLUGINS
            \"${dest_prefix}/lib/modules/*${CMAKE_SHARED_MODULE_SUFFIX}\")
        file(GLOB_RECURSE SPROKIT_PLUGINS
            \"${dest_prefix}/lib/sprokit/*${CMAKE_SHARED_MODULE_SUFFIX}\")
        file(GLOB_RECURSE LOGGER_PLUGINS
            \"${dest_prefix}/${plugin_dest_dir}/vital_*_logger${CMAKE_SHARED_LIBRARY_SUFFIX}\")
        set(BU_CHMOD_BUNDLE_ITEMS ON)
        set(ENV{VS_UNICODE_OUTPUT} \"\")
        include(${MAPTK_SOURCE_DIR}/CMake/utils/maptk-utils-bundle.cmake)
        include( ${MAPTK_BINARY_DIR}/CMakeFiles/plugin-libs-\${CMAKE_INSTALL_CONFIG_NAME}.cmake )
        list(APPEND FIXUP_DIRS ${FIXUP_DIRS} \${bundle_path})
        list(REMOVE_DUPLICATES FIXUP_DIRS)
        set(libs \${KWIVER_PLUGINS} \${SPROKIT_PLUGINS} \${LOGGER_PLUGINS} \${QTPLUGINS})
        include(BundleUtilities)
        fixup_bundle(\"${gui-exe}\" \"\${libs}\" \"\${FIXUP_DIRS}\")
        ")

  if (WIN32)
    include(InstallRequiredSystemLibraries)
  endif()

endif(MAPTK_FIXUP_BUNDLE_ON_PACKAGE)
