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
  WorldView.ui
)

set(gui_ui
  ${gui_am_ui}
  CameraOptions.ui
  FeatureOptions.ui
)

set(gui_resources
  icons/icons.qrc
)

set(gui_moc_headers
  ActorColorButton.h
  CameraOptions.h
  CameraView.h
  FeatureOptions.h
  MainWindow.h
  WorldView.h
)

set(gui_sources
  ActorColorButton.cxx
  CameraOptions.cxx
  CameraView.cxx
  FeatureOptions.cxx
  Project.cxx
  MainWindow.cxx
  WorldView.cxx
  main.cxx
  vtkMaptkCamera.cxx
  vtkMaptkCameraRepresentation.cxx
)

qt4_wrap_ui(gui_ui_sources ${gui_ui})
qt4_wrap_cpp(gui_moc_sources ${gui_moc_headers})
qt4_add_resources(gui_res_sources ${gui_resources})

qte_amc_wrap_ui(gui_amc_sources ActionManagerDialog ${gui_am_ui})

maptk_add_executable(mapgui
  WIN32
  ${gui_sources}
  ${gui_ui_sources}
  ${gui_moc_sources}
  ${gui_res_sources}
  ${gui_amc_sources}
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
  ${QT_QTMAIN_LIBRARY}
  ${QT_LIBRARIES}
)

# add DESTDIR switch (Turn it on if not WIN32?)
# do components bit (don't install hedader files)')

set(gui-exe "\${DESTDIR}\$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}/bin/mapgui${CMAKE_EXECUTABLE_SUFFIX}")

# Add expected location of VTK dlls
list(APPEND FIXUP_DIRS "${VTK_INSTALL_PREFIX}/bin")

# For now, we're assuming we're pulling qtextensions from a build directory
# (not an install).  Also, Mac and/or Linus will need be a bit different,
# but will be handled later (they'd likely use CMAKE_BUILD_TYPE instead
# of CMAKE_CONFIGURATION_TYPES). For Windows really expect the dll will
# be in Release subdirectory, and so look there unless
# CMAKE_CONFIGURATION_TYPES indicates only "debug"
string(TOLOWER "${CMAKE_CONFIGURATION_TYPES}" BUILD_TYPE)
if(BUILD_TYPE STREQUAL "debug")
  set(BINARY_DIR "Debug")
else()
  set(BINARY_DIR "Release")
endif()
list(APPEND FIXUP_DIRS "${qtExtensions_DIR}/${BUILD_TYPE}")

# Location for QT dlls
list(APPEND FIXUP_DIRS "${QT_BINARY_DIR}")

set(FIXUP_LIBS "")

install(CODE "
        set(ENV{VS_UNICODE_OUTPUT} \"\")
        include(BundleUtilities)
        fixup_bundle(\"${gui-exe}\" \"${FIXUP_LIBS}\" \"${FIXUP_DIRS}\")
        "
)

if (WIN32)
  include(InstallRequiredSystemLibraries)
endif()
