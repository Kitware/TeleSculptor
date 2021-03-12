set(CMAKE_AUTOMOC ON)
set(Qt5_MODULES Core Designer UiPlugin Widgets Svg Xml)
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
  vtkIOGeometry
  vtkIOImage
  vtkIOPLY
  vtkIOXML
  vtkImagingCore
  vtkInteractionStyle
  vtkInteractionWidgets
  vtkRenderingAnnotation
  vtkRenderingFreeType
  )
if(VTK_VERSION VERSION_LESS 8.2)
  message(FATAL_ERROR "${PROJECT_NAME} supports VTK >= v8.2 "
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
option( TELESCULPTOR_FIXUP_BUNDLE_ON_PACKAGE
  "Perform bundle fixup when building a package. Only set when packaging."
  OFF )

include_directories(
  ${CMAKE_CURRENT_BINARY_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${TELESCULPTOR_SOURCE_DIR}
  ${TELESCULPTOR_BINARY_DIR}
)

set(gui_am_ui
  CameraView.ui
  GroundControlPointsView.ui
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
  LoggerView.ui
  PointOptions.ui
  RulerOptions.ui
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
  GroundControlPointsModel.cxx
  GroundControlPointsView.cxx
  GroundControlPointsWidget.cxx
  GuiCommon.cxx
  ImageOptions.cxx
  LoggerView.cxx
  MainWindow.cxx
  MatchMatrixAlgorithms.cxx
  MatchMatrixWindow.cxx
  MetadataView.cxx
  PointOptions.cxx
  Project.cxx
  RulerHelper.cxx
  RulerOptions.cxx
  RulerWidget.cxx
  Utils.cxx
  VideoImport.cxx
  VolumeOptions.cxx
  WorldView.cxx
  main.cxx
  vtkMaptkAxesActor.cxx
  vtkMaptkCamera.cxx
  vtkMaptkCameraRepresentation.cxx
  vtkMaptkDistanceRepresentation2D.cxx
  vtkMaptkDistanceWidget.cxx
  vtkMaptkFeatureTrackRepresentation.cxx
  vtkMaptkImageDataGeometryFilter.cxx
  vtkMaptkImageUnprojectDepth.cxx
  vtkMaptkInteractorStyle.cxx
  vtkMaptkPointHandleRepresentation3D.cxx
  vtkMaptkPointPicker.cxx
  vtkMaptkPointPlacer.cxx
  vtkMaptkScalarDataFilter.cxx
  vtkMaptkScalarsToGradient.cxx
  vtkMaptkSeedWidget.cxx
  tools/AbstractTool.cxx
  tools/BundleAdjustTool.cxx
  tools/CanonicalTransformTool.cxx
  tools/ComputeAllDepthTool.cxx
  tools/ComputeDepthTool.cxx
  tools/FuseDepthTool.cxx
  tools/InitCamerasLandmarksTool.cxx
  tools/MeshColoration.cxx
  tools/NeckerReversalTool.cxx
  tools/RunAllTool.cxx
  tools/SaveFrameTool.cxx
  tools/SaveKeyFrameTool.cxx
  tools/TrackFeaturesSprokitTool.cxx
  tools/TrackFeaturesTool.cxx
  tools/TrackFilterTool.cxx
  tools/TriangulateTool.cxx
)

if (APPLE)
  find_library(AppKit AppKit)
  list(APPEND gui_sources
    Utils.mm
  )
endif()

telesculptor_use_appdata(
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
  ${AppKit}
  ${VTK_LIBRARIES}
  ${QT_LIBRARIES}
)

if(vtkWebGLExporter_LOADED)
  message(STATUS "Module vtkWebGLExporter present: wekGL export enabled")
  target_compile_definitions(TeleSculptor PRIVATE -DVTKWEBGLEXPORTER)
  target_link_libraries(TeleSculptor vtkWebGLExporter)
endif()

if(APPLE)
  file(STRINGS "${TELESCULPTOR_SOURCE_DIR}/LICENSE" copyright_line
    LIMIT_COUNT 1 REGEX "Copyright")

  set_target_properties(TeleSculptor PROPERTIES
    OUTPUT_NAME TeleSculptor
    MACOSX_BUNDLE_NAME "TeleSculptor"
    MACOSX_BUNDLE_INFO_STRING "TeleSculptor v${TeleSculptor_VERSION} Application"
    MACOSX_BUNDLE_SHORT_VERSION_STRING "${TeleSculptor_VERSION}"
    MACOSX_BUNDLE_COPYRIGHT "${copyright_line}"
    )
endif()

kwiver_install(TARGETS TeleSculptor
  ${exports}
  RUNTIME DESTINATION bin
  BUNDLE DESTINATION "${CMAKE_BUNDLE_LOCATION}")

# Set destination paths for plugins and the GUI executable
set(plugin_dest_dir lib)
set(qtconf_dest_dir bin)
set(qtplugin_dest_dir "bin/plugins")
set(dest_prefix "\${DESTDIR}\$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}")
set(gui-exe "${dest_prefix}/bin/TeleSculptor${CMAKE_EXECUTABLE_SUFFIX}")
if(WIN32)
  set(plugin_dest_dir bin)
endif()
if(APPLE)
  set(gui-exe "${dest_prefix}/MacOS/TeleSculptor")
  set(qtplugin_dest_dir "plugins")
  set(qtconf_dest_dir Resources)
endif()

#
# Do bundle fixup
#    This could be conditional on APPLE || WIN32 also
#
if (TELESCULPTOR_FIXUP_BUNDLE_ON_PACKAGE)
  include(${TELESCULPTOR_SOURCE_DIR}/CMake/telesculptor-install-qt-utils.cmake)

  if (WIN32)
    # Add expected location of VTK dlls
    list(APPEND FIXUP_DIRS "${VTK_INSTALL_PREFIX}/bin")
  else()
    # Add expected location of VTK libraries
    list(APPEND FIXUP_DIRS "${VTK_INSTALL_PREFIX}/lib")
  endif()

  # For Windows expect the dll will
  # be in Release subdirectory, and so look there unless
  # CMAKE_CONFIGURATION_TYPES indicates only "debug"
  if(CMAKE_GENERATOR_IS_MULTI_CONFIG AND CMAKE_CONFIGURATION_TYPES)
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
      file(GLOB VC_DIRS "${fletch_ROOT}/x64/vc*/bin")
      list(APPEND FIXUP_DIRS ${VC_DIRS})
    else()
      list(APPEND FIXUP_DIRS "${fletch_ROOT}/lib")
    endif()
  endif()

  if(DEFINED CUDA_TOOLKIT_ROOT_DIR)
    if(WIN32)
      list(APPEND FIXUP_DIRS "${CUDA_TOOLKIT_ROOT_DIR}/bin")
    else()
      list(APPEND FIXUP_DIRS "${CUDA_TOOLKIT_ROOT_DIR}/lib")
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

  install(DIRECTORY "${KWIVER_DIR}/lib/${BINARY_DIR}/kwiver/modules"
          DESTINATION lib/kwiver
          COMPONENT Runtime
          FILES_MATCHING PATTERN "*plugin${CMAKE_SHARED_MODULE_SUFFIX}" )

  install(DIRECTORY "${KWIVER_DIR}/lib/${BINARY_DIR}/kwiver/modules/applets"
          DESTINATION lib/kwiver/modules
          COMPONENT Runtime
          FILES_MATCHING PATTERN "*${CMAKE_SHARED_MODULE_SUFFIX}" )

  install(DIRECTORY "${KWIVER_DIR}/lib/${BINARY_DIR}/kwiver/modules/"
          DESTINATION ${plugin_dest_dir}
          COMPONENT Runtime
          FILES_MATCHING PATTERN "*logger${CMAKE_SHARED_MODULE_SUFFIX}"
          PATTERN "applets" EXCLUDE
          PATTERN "plugin_explorer" EXCLUDE )

  install(DIRECTORY "${KWIVER_DIR}/lib/${BINARY_DIR}/kwiver/processes"
          DESTINATION lib/kwiver
          COMPONENT Runtime
          FILES_MATCHING PATTERN "*${CMAKE_SHARED_MODULE_SUFFIX}" )

  install(DIRECTORY "${KWIVER_DIR}/share/kwiver/${KWIVER_VERSION}/config"
          DESTINATION share/kwiver/${KWIVER_VERSION}
          COMPONENT Runtime )

  install(DIRECTORY "${fletch_ROOT}/share/proj"
          DESTINATION share COMPONENT Runtime)

  install(DIRECTORY "${fletch_ROOT}/share/gdal"
          DESTINATION share COMPONENT Runtime)

  if(${TELESCULPTOR_ENABLE_TOOLS})
    set(kwiver_tools
        kwiver
        plugin_explorer)

    set(KWIVER_BIN_DIR "${KWIVER_DIR}/bin")
    if(WIN32)
      set(KWIVER_BIN_DIR "${KWIVER_BIN_DIR}/${BINARY_DIR}")
    endif()
    foreach(tool ${kwiver_tools})
      set(tool_path "${KWIVER_BIN_DIR}/${tool}${CMAKE_EXECUTABLE_SUFFIX}")
      if(EXISTS "${tool_path}")
        install(PROGRAMS "${tool_path}"
                DESTINATION bin
                COMPONENT Runtime)
      else()
        message(WARNING "Could not find ${tool} at ${tool_path}")
      endif()
    endforeach()
  endif()

  get_property( bundle_path GLOBAL PROPERTY telesculptor_bundle_paths)

  list(APPEND FIXUP_DIRS "${VTK_CMAKE_DIR}../..")

  set(QT5_MODULES "Gui;Widgets")
  install_qt_plugins(QT5_MODULES "${qtplugin_dest_dir}" QT_PLUGINS)
  # install a qt.conf file
  install(CODE "
    file(WRITE \"${dest_prefix}/${qtconf_dest_dir}/qt.conf\" \"\")
    " COMPONENT Runtime)

  set( script_file  ${TELESCULPTOR_BINARY_DIR}/CMakeFiles/plugin-libs-$<CONFIGURATION>.cmake)

  file(GENERATE OUTPUT ${script_file}
    CONTENT "
set(bundle_path \"${bundle_path}\")
          ")

  install(CODE "
        set(QT_PLUGINS_Release \"${QT_PLUGINS_Release}\")
        set(QT_PLUGINS_Debug \"${QT_PLUGINS_Debug}\")
        string(CONCAT QTPLUGINS \"QT_PLUGINS_\" \"\${CMAKE_INSTALL_CONFIG_NAME}\")
        file(GLOB_RECURSE QTPLUGINS
            \"${dest_prefix}/${qtplugin_dest_dir}/*${CMAKE_SHARED_LIBRARY_SUFFIX}\")
        file(GLOB_RECURSE KWIVER_PLUGINS
            \"${dest_prefix}/lib/kwiver/modules/*${CMAKE_SHARED_MODULE_SUFFIX}\")
        file(GLOB_RECURSE SPROKIT_PLUGINS
            \"${dest_prefix}/lib/kwiver/processes/*${CMAKE_SHARED_MODULE_SUFFIX}\")
        file(GLOB_RECURSE LOGGER_PLUGINS
            \"${dest_prefix}/${plugin_dest_dir}/vital_*_logger${CMAKE_SHARED_LIBRARY_SUFFIX}\")
        set(BU_CHMOD_BUNDLE_ITEMS ON)
        set(ENV{VS_UNICODE_OUTPUT} \"\")
        include(${TELESCULPTOR_SOURCE_DIR}/CMake/utils/telesculptor-utils-bundle.cmake)
        include( ${TELESCULPTOR_BINARY_DIR}/CMakeFiles/plugin-libs-\${CMAKE_INSTALL_CONFIG_NAME}.cmake )
        list(APPEND FIXUP_DIRS ${FIXUP_DIRS} \${bundle_path})
        list(REMOVE_DUPLICATES FIXUP_DIRS)
        set(libs \${KWIVER_PLUGINS} \${SPROKIT_PLUGINS} \${LOGGER_PLUGINS} \${QTPLUGINS})
        include(BundleUtilities)
        fixup_bundle(\"${gui-exe}\" \"\${libs}\" \"\${FIXUP_DIRS}\")

        if(UNIX AND NOT APPLE)
          # fixup_bundle puts all libraries in \"bin\" on Linux.
          # we want them in \"lib\" instead, so move them
          file(GLOB dyn_libs \"${dest_prefix}/bin/*.so*\")
          foreach(dyn_lib \${dyn_libs})
            get_filename_component(fname \${dyn_lib} NAME)
            message(STATUS \"Moving bin->lib : \${fname}\")
            file(RENAME \${dyn_lib} \"${dest_prefix}/lib/\${fname}\")
          endforeach()
          # Now fixup the RPATH of libraries on Linux
          find_program(patchelf_bin patchelf)
          if(NOT patchelf_bin)
            message(WARNING \"Could not find patchelf binary, unable to fix RPATHs\")
          else()
            file(GLOB dyn_libs \"${dest_prefix}/lib/*.so*\")
            foreach(dyn_lib \${dyn_libs})
              message(STATUS \"Setting RPATH to '\\\$ORIGIN/../lib' in \${dyn_lib}\")
              execute_process(COMMAND \${patchelf_bin} --set-rpath \\\$ORIGIN/../lib \${dyn_lib})
            endforeach()
            file(GLOB plugins \"${dest_prefix}/bin/plugins/*/*.so*\"
                              \"${dest_prefix}/lib/kwiver/*/*.so*\")
            foreach(plugin \${plugins})
              message(STATUS \"Setting RPATH to '\\\$ORIGIN/../../../lib' in \${plugin}\")
              execute_process(COMMAND \${patchelf_bin} --set-rpath \\\$ORIGIN/../../../lib \${plugin})
            endforeach()
            file(GLOB plugins \"${dest_prefix}/lib/kwiver/*/*/*.so*\")
            foreach(plugin \${plugins})
              message(STATUS \"Setting RPATH to '\\\$ORIGIN/../../../../lib' in \${plugin}\")
              execute_process(COMMAND \${patchelf_bin} --set-rpath \\\$ORIGIN/../../../../lib \${plugin})
            endforeach()
          endif()
        endif()
        ")

  if (WIN32)
    include(InstallRequiredSystemLibraries)
  endif()

endif(TELESCULPTOR_FIXUP_BUNDLE_ON_PACKAGE)
