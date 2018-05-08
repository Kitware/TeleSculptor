find_package(Qt4 4.8 REQUIRED)
include(${QT_USE_FILE})

find_package(qtExtensions REQUIRED)
include(${qtExtensions_USE_FILE})

find_package(VTK 8.0 REQUIRED
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

set(gui_moc_headers
  AboutDialog.h
  ActorColorButton.h
  CameraOptions.h
  CameraView.h
  ColorizeSurfaceOptions.h
  DataColorOptions.h
  DataFilterOptions.h
  DepthMapFilterOptions.h
  DepthMapOptions.h
  DepthMapView.h
  DepthMapViewOptions.h
  FeatureOptions.h
  GradientSelector.h
  ImageOptions.h
  MainWindow.h
  MatchMatrixWindow.h
  PointOptions.h
  Project.h
  VideoImport.h
  VolumeOptions.h
  WorldView.h
  tools/AbstractTool.h
  tools/BundleAdjustTool.h
  tools/CanonicalTransformTool.h
  tools/ComputeDepthTool.h
  tools/InitCamerasLandmarksTool.h
  tools/NeckerReversalTool.h
  tools/SaveFrameTool.h
  tools/SaveKeyFrameTool.h
  tools/TrackFilterTool.h
  tools/TrackFeaturesSprokitTool.h
  tools/TrackFeaturesTool.h
  tools/TriangulateTool.h
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
  GuiCommon.cxx
  ImageOptions.cxx
  MainWindow.cxx
  MatchMatrixAlgorithms.cxx
  MatchMatrixWindow.cxx
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
  tools/AbstractTool.cxx
  tools/BundleAdjustTool.cxx
  tools/CanonicalTransformTool.cxx
  tools/ComputeDepthTool.cxx
  tools/InitCamerasLandmarksTool.cxx
  tools/NeckerReversalTool.cxx
  tools/MeshColoration.cxx
  tools/ReconstructionData.cxx
  tools/SaveFrameTool.cxx
  tools/SaveKeyFrameTool.cxx
  tools/TrackFilterTool.cxx
  tools/TrackFeaturesSprokitTool.cxx
  tools/TrackFeaturesTool.cxx
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

qt4_wrap_ui(gui_ui_sources ${gui_ui})
qt4_wrap_cpp(gui_moc_sources ${gui_moc_headers})
qt4_add_resources(gui_res_sources ${gui_resources})

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
  ${gui_moc_sources}
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
  ${QT_QTMAIN_LIBRARY}
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
  list(APPEND FIXUP_DIRS "${QT_BIN_DIR}")
  list(APPEND FIXUP_DIRS "${QT_LIBRARY_DIR}")
  list(APPEND FIXUP_DIRS "${QT_BINARY_DIR}")

  install(DIRECTORY "${KWIVER_DIR}/lib/${BINARY_DIR}/modules" DESTINATION lib
          COMPONENT Runtime
          FILES_MATCHING PATTERN "*plugin${CMAKE_SHARED_MODULE_SUFFIX}" )

  install(DIRECTORY "${KWIVER_DIR}/lib/${BINARY_DIR}/modules/"
          DESTINATION ${plugin_dest_dir}
          COMPONENT Runtime
          FILES_MATCHING PATTERN "*logger${CMAKE_SHARED_MODULE_SUFFIX}"
          PATTERN "plugin_explorer" EXCLUDE )

  install(DIRECTORY "${KWIVER_DIR}/lib/${BINARY_DIR}/sprokit" DESTINATION lib
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

  install(DIRECTORY "${QT_PLUGINS_DIR}/imageformats" DESTINATION ${plugin_dest_dir}/plugins COMPONENT Runtime)
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
        file(GLOB_RECURSE QTPLUGINS
            \"${dest_prefix}/${plugin_dest_dir}/plugins/*${CMAKE_SHARED_LIBRARY_SUFFIX}\")
        file(GLOB_RECURSE KWIVER_PLUGINS
            \"${dest_prefix}/lib/modules/*${CMAKE_SHARED_MODULE_SUFFIX}\")
        file(GLOB_RECURSE LOGGER_PLUGINS
            \"${dest_prefix}/${plugin_dest_dir}/vital_*_logger${CMAKE_SHARED_LIBRARY_SUFFIX}\")
        set(BU_CHMOD_BUNDLE_ITEMS ON)
        set(ENV{VS_UNICODE_OUTPUT} \"\")
        include(${MAPTK_SOURCE_DIR}/CMake/utils/maptk-utils-bundle.cmake)
        include( ${MAPTK_BINARY_DIR}/CMakeFiles/plugin-libs-\${CMAKE_INSTALL_CONFIG_NAME}.cmake )
        list(APPEND FIXUP_DIRS ${FIXUP_DIRS} \${bundle_path})
        list(REMOVE_DUPLICATES FIXUP_DIRS)
        set(libs \${KWIVER_PLUGINS} \${LOGGER_PLUGINS} \${QTPLUGINS})
        include(BundleUtilities)
        fixup_bundle(\"${gui-exe}\" \"\${libs}\" \"\${FIXUP_DIRS}\")
        ")

  if (WIN32)
    include(InstallRequiredSystemLibraries)
  endif()

endif(MAPTK_FIXUP_BUNDLE_ON_PACKAGE)
