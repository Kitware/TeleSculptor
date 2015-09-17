find_package(Qt4 4.8 REQUIRED)
include(${QT_USE_FILE})

find_package(qtExtensions REQUIRED)

find_package(VTK 6.2 REQUIRED)
include(${VTK_USE_FILE})

include_directories(
  ${CMAKE_CURRENT_BINARY_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${MAPTK_SOURCE_DIR}
  ${MAPTK_BINARY_DIR}
)

set(gui_ui
  MainWindow.ui
)

set(gui_resources
)

set(gui_moc_headers
  CameraView.h
  MainWindow.h
  WorldView.h
)

set(gui_sources
  CameraView.cxx
  Project.cxx
  MainWindow.cxx
  WorldView.cxx
  main.cxx
  vtkMkCamera.cxx
)

qt4_wrap_ui(gui_ui_sources ${gui_ui})
qt4_wrap_cpp(gui_moc_sources ${gui_moc_headers})
qt4_add_resources(gui_res_sources ${gui_resources})

maptk_add_executable(mapgui
  ${gui_sources}
  ${gui_ui_sources}
  ${gui_moc_sources}
  ${gui_res_sources}
)

target_link_libraries(mapgui
  maptk
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

maptk_add_executable(sandbox
  vtkSandbox.cxx
)

target_link_libraries(sandbox
  ${Boost_SYSTEM_LIBRARY}
  ${Boost_FILESYSTEM_LIBRARY}
  maptk
  vtkRenderingFreeTypeOpenGL
  vtkRenderingOpenGL
  vtkRenderingVolumeOpenGL
  vtkInteractionStyle
  vtkViewsCore
  vtksys
  vtkCommonCore
  vtkRenderingCore
)
