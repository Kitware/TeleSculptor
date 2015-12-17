find_package(Qt4 4.8 REQUIRED)
include(${QT_USE_FILE})

find_package(qtExtensions REQUIRED)
include(${qtExtensions_USE_FILE})

find_package(VTK 6.2 REQUIRED)
include(${VTK_USE_FILE})

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

maptk_configure_file(version.h
  "${CMAKE_CURRENT_SOURCE_DIR}/Version.h.in"
  "${CMAKE_CURRENT_BINARY_DIR}/Version.h"
  MAPTK_VERSION
  MAPTK_COPYRIGHT_YEAR
)

qt4_wrap_ui(gui_ui_sources ${gui_ui})
qt4_wrap_cpp(gui_moc_sources ${gui_moc_headers})
qt4_add_resources(gui_res_sources ${gui_resources})

qte_amc_wrap_ui(gui_amc_sources ActionManagerDialog ${gui_am_ui})

maptk_add_executable(mapgui
  ${gui_sources}
  ${gui_ui_sources}
  ${gui_moc_sources}
  ${gui_res_sources}
  ${gui_amc_sources}
)

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
  ${QT_LIBRARIES}
)
