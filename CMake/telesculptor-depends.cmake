###
# Find all package dependencies
#

message(STATUS "Looking for kwiver in : ${kwiver_DIR}")

find_package(kwiver REQUIRED)
if ( IS_DIRECTORY ${fletch_DIR} )
  find_package(fletch REQUIRED)
  message(STATUS "Using fletch from : ${fletch_DIR}")
endif()
include( kwiver-cmake-future )
include( kwiver-utils )
include( kwiver-flags )
include( kwiver-configcheck )
include( telesculptor-utils ) # local utilities

# We must find third party packages used by both KWIVER and TeleSculptor here
# otherwise CMake will be unable to find those targets.
# In the future, this should be exported by KWIVER.

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
