===============================================================================
  MAP-Tk GUI
===============================================================================

.. contents::

Overview
========

The MAP-Tk GUI provides a means of visualizing the output of MAP-Tk data and
results. Entities available for display include the cameras, camera imagery,
feature points, and landmarks.

The GUI consists of a primary (world) view, a secondary (camera) view, and a
camera selection panel. The world view is a three dimensional view that shows
the cameras and landmarks in the computed world coordinate system. The camera
view shows the imagery from a single camera along with corresponding feature
points, projected landmarks, and estimation residuals. The camera selection
panel provides controls to select the active camera and to use the slide show
mode. The camera view and camera selection panel are dockable and can be
rearranged or detached to suit user preference.

World View
==========

The world view provides a three dimensional view showing the computed and all
of the cameras in a single, homogeneous coordinate space that is especially
useful for seeing the landmarks in 3D, and seeing the spatial relations between
cameras, and between cameras and landmarks.

Landmarks are displayed as points, while cameras are displayed as frustums of
semi-arbitrary length, with the active camera highlighted (see
`Camera Options`_). The "camera path", which is a line connecting the camera
centers, can also be displayed.

.. TODO update above when we add support for cameras as points

Tool Bar
--------

:icon:`view-reset` Reset View
  Resets the view extents so that the entire scene is visible. Additional
  actions are available via the action's associated pop-up.

:icon:`blank` Zoom to Landmarks
  Resets the view extents so that all landmarks are visible. This action is
  available via the `Reset View` button's associated pop-up menu.

:icon:`camera` Show Cameras
  Toggles visibility of cameras and related visualizations. The associated
  pop-up provides additional options; see `Camera Options`_.

:icon:`landmark` Show Landmarks
  Toggles visibility of landmarks. The associated pop-up provides additional
  options; see `Feature Options`_.

Camera Options
--------------

The :action:`camera Show Cameras` pop-up provides additional controls that can
be used to control the display of the cameras in the world view. These allow
changing the color of both the active and inactive cameras as well as the
camera path, changing the size of the camera frustums, and toggling visibility
of the inactive cameras and camera path separate from the overall camera
visibility.

The camera scale controls are logarithmic, and are relative to a "base size"
that is computed from the scene data. (This is used to minimize the perceptual
difference in camera frustum size relative to the numerical scale of the data,
which can be arbitrary, and significantly different across various data sets.)
The inactive camera scale is relative to the active camera scale, with the
maximum allowed value giving active and inactive camera frustums the same size.

.. notice::
  Display of inactive cameras as points is not yet implemented. Selecting this
  option has no effect, and may cause display glitches.

.. TODO remove above notice and fix documentation when we support cameras as
   points

Feature Options
---------------

The :action:`landmark Show Landmarks` pop-up provides additional controls that
can be used to control the display of the landmarks in the world view. (The
same controls are also used in the camera view to manipulate the display of
feature points and landmarks in that view.) These allow the color of the
items to be changed, as well as their size. Feature items (that is, feature
points and landmarks) are displayed as dots, with a fixed size-on-screen that
is independent of the view.

Camera View
===========

The camera view provides a camera space view of detected feature points and
computed landmarks (projected to the camera space), as well as the
corresponding input imagery, for the active camera. Additionally, the
estimation residuals |--| the difference between landmarks and feature points
which participated in computing their estimated positions |--| can be
displayed as line segments between the feature point location and projected
landmark location.

Tool Bar
--------

:icon:`view-reset` Reset View
  Resets the view to the camera image extents. Additional actions are available
  via the action's associated pop-up.

:icon:`blank` Zoom Extents
  Resets the view extents so that the entire scene is visible. This action is
  available via the `Reset View` button's associated pop-up menu.

:icon:`feature` Show Feature Points
  Toggles visibility of feature points. The associated pop-up provides
  additional options; see `Feature Options`_.

:icon:`landmark` Show Landmarks
  Toggles visibility of landmarks. The associated pop-up provides additional
  options; see `Feature Options`_.

:icon:`residual` Show Residuals
  Toggles visibility of the landmark estimation residuals. The associated
  pop-up allows the color of the displayed residuals to be changed.

Camera Selection
================

The camera selection panel contains a large slider used to select the active
camera. The active camera is highlighted in the world view, and used to control
which camera's imagery and feature points are displayed in the camera view. A
spin box next to the slider shows the active camera number, and can also be
used to select the active camera.

The controls to the right of the panel control the application's slide show
mode. Slide show mode automatically increments through the loaded cameras at a
fixed rate. This can be used to view the feature points for each camera / input
image in sequence. Setting the delay between cameras sufficiently low can be
used to simulate video playback for image sequences taken from a motion imagery
source.

The slide show action controls are also available via the `View <#view-menu>`_
menu. The small slider controls the delay between slides. The slider response
is logarithmic, with single steps in one-tenth powers of ten. The slider tool
tip includes the current delay in human readable units.

Data Files
==========

The most convenient way to load data is to open the configuration file
(``.conf``) that is provided to the bundle adjustment tool. This file specifies
the locations of all relevant data and outputs, including camera KRTD files,
imagery, feature tracks and landmarks. It is also possible to load individual
cameras (via their KRTD files), track files, and landmark files.

.. notice::
  When loading cameras individually, camera identifiers are assigned
  sequentially based on the order in which cameras are loaded. In order for
  feature points to be correctly associated with their corresponding cameras,
  the cameras must be loaded so that these automatically assigned identifies
  match those that were assigned by the feature detection pipeline.

Menu
====

File Menu
---------

:icon:`open` Open
  Presents a dialog that allows the selection of one or more data files to be
  loaded into the session.

:icon:`quit` Quit
  Exits the application.

View Menu
---------

:icon:`playback-play` Play Slideshow
  Toggles playback of the slide show.

:icon:`playback-loop` Loop Slideshow
  Toggles if the slideshow should restart from the beginning after the last
  camera. When disabled, the slideshow ends when the last camera becomes
  active.

.. |--| unicode:: U+02014 .. em dash
