find_package(Qt4 4.8 REQUIRED)
include(${QT_USE_FILE})

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
  MainWindow.h
)

set(gui_sources
  MainWindow.cxx
  main.cxx
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
  vtkRenderingOpenGL
  vtkRenderingFreeTypeOpenGL
  vtkRenderingVolumeOpenGL
  vtkGUISupportQt
  ${QT_LIBRARIES}
)
