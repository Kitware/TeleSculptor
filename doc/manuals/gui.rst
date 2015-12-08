===============================================================================
  MAP-Tk GUI
===============================================================================

.. role:: f
   :class: math

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
panel provides controls to select the active camera and to use the slideshow
mode. The camera view and camera selection panel are dockable and can be
rearranged or detached to suit user preference.

World View
==========

The world view provides a three dimensional view showing the computed landmarks
and all of the cameras in a single, homogeneous coordinate space that is
especially useful for seeing the landmarks in 3D, and seeing the spatial
relations between cameras, and between cameras and landmarks.

Landmarks are displayed as points, while cameras are displayed as frustums of
semi-arbitrary length, with the active camera highlighted (see
`Camera Options`_). The "camera path", which is a line connecting the camera
centers, can also be displayed.

.. TODO update above when we add support for cameras as points

Additionally, the world view can display a representation of the ground plane
(i.e. the world coordinate plane :f:`z = 0`), and can display the frame image
for the active camera projected to the same.

Tool Bar
--------

:icon:`view-reset` Reset View
  Resets the view extents so that the entire scene is visible. Additional
  actions are available via the action's associated pop-up.

:icon:`blank` Zoom to Landmarks
  Resets the view extents so that all landmarks are visible. This action is
  available via the :action:`view-reset Reset View` button's associated pop-up
  menu.

:icon:`blank` View to World Top/Left/Right/Front/Back
  Resets the view rotation to a "standard" rotation, such that the view axes
  are parallel with the world axes. These actions are available via the
  :action:`view-reset Reset View` button's associated pop-up menu.

:icon:`blank` Perspective
  Toggles the world view between perspective and parallel projection.
  Perspective projection more closely models human vision and is often useful
  for visualizing depth in the scene. Parallel projection can be useful for
  viewing the scene in profile. This action is available via the
  :action:`view-reset Reset View` button's associated pop-up menu.

:icon:`image` Show Camera Frame Image
  Toggles visibility of the projected camera frame image. The associated
  pop-up allows the opacity of the same to be adjusted.

:icon:`camera` Show Cameras
  Toggles visibility of cameras and related visualizations. The associated
  pop-up provides additional options; see `Camera Options`_.

:icon:`landmark` Show Landmarks
  Toggles visibility of landmarks. The associated pop-up provides additional
  options; see `Point Options`_.

:icon:`grid` Show Ground Plane Grid
  Toggles visibility of the ground plane. The ground plane is the :f:`z = 0`
  plane in world coordinates. The grid is centered about :f:`x = y = 0`,
  however the grid lines are otherwise strictly aesthetic and do not correspond
  to any particular values.

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

Point Options
-------------

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

:icon:`image` Show Camera Frame Image
  Toggles visibility of the camera frame image. The associated pop-up allows
  the opacity of the same to be adjusted.

:icon:`feature` Show Feature Points
  Toggles visibility of feature points / trails. The associated pop-up provides
  additional options; see `Feature Options`_.

:icon:`landmark` Show Landmarks
  Toggles visibility of landmarks. The associated pop-up provides additional
  options; see `Point Options`_.

:icon:`residual` Show Residuals
  Toggles visibility of the landmark estimation residuals. The associated
  pop-up allows the color of the displayed residuals to be changed.

Feature Options
---------------

In addition to active feature points, which have all the options described in
`Point Options`_, the position of feature points on adjacent frames may also be
displayed by enabling :action:`- Trails`. For image collections where cameras
adjacent in the camera list are also spatially similar (especially when using
consecutive video frames as input), these may be useful as an additional means
of visualizing camera motion.

The trail color and length (number of adjacent frames to be used) may be
changed, as well as whether to show trails only for lower-numbered frames
("historic" mode), or for all adjacent frames ("symmetric" mode). In all cases,
trails are displayed only for active feature points.

Camera Selection
================

The camera selection panel contains a large slider used to select the active
camera. The active camera is highlighted in the world view, and used to control
which camera's imagery and feature points are displayed in the camera view. A
spin box next to the slider shows the active camera number, and can also be
used to select the active camera.

The controls to the right of the panel control the application's slideshow
mode. Slideshow mode automatically increments through the loaded cameras at a
fixed rate. This can be used to view the feature points for each camera / input
image in sequence. Setting the delay between cameras sufficiently low can be
used to simulate video playback for image sequences taken from a motion imagery
source.

The slideshow action controls are also available via the `View <#view-menu>`_
menu. The small slider controls the delay between slides. The slider response
is logarithmic, with single steps in one-tenth powers of ten. The slider tool
tip includes the current delay in human readable units.

Match Matrix View
=================

The match matrix view provides a visualization of the feature point
associations across camera frames. Pixels in the image correspond to values in
the "match matrix" representing the number of feature points that feature
detection has determined correspond to the same real world feature. Several
options are provided to adjust the visualization:

* Layout controls the position of "identity" values, i.e. values that compare a
  frame to itself rather than a distinct frame. The default, "diagonal", simply
  maps the frame number directly to both the :f:`X` and :f:`Y` axes.
  "Horizontal" skews the image so that the :f:`y` values are relative to the
  "identity" values, placing them in a horizontal line at :f:`y = 0`, with
  positive :f:`y` representing "later" frames, and negative :f:`y` representing
  "earlier" frames. "Vertical" reverses these axes.

* Orientation controls which screen direction is considered positive :f:`Y`.
  The default, "matrix", uses down for positive :f:`Y`, as in textual value
  tables (e.g. textual listings of matrices, spreadsheets) or images. "Graph"
  uses up for positive :f:`Y`, as in most graphical plots.

* Values controls what values are used for each pixel. The default, "absolute",
  uses the raw number of feature point correlations (which, for "identity"
  values is equal to the total number of feature points on that frame).
  "Relative (combined)" mode uses the percent of common feature points relative
  to the total number of distinct feature points on each frame being compared.
  The other two "relative" modes give the percent relative to the total number
  of feature points for the frame represented by either the :f:`X` or :f:`Y`
  axis.

* Scale controls the scaling function that is applied to the values produced
  according to the value mode. The choices are "linear", "logarithmic" and
  "exponential", and should be self explanatory. In absolute value mode,
  logarithmic scale uses the maximum value as the logarithm base. Otherwise,
  the base can be adjusted with the "range" control, which applies a pre-scale
  to the value before computing the logarithm (thereby allowing the shape of
  the scaling curve to be adjusted). Exponential scale allows the user to
  select the exponent.

* Color provides the set of colors to which scaled values are mapped. Several
  presets are available according to user taste. Different presets may help
  emphasize different aspects of the data.

Moving the mouse over the image will display which frames are being compared
and the number or percentage of feature correlations in the status bar. The
match matrix view also allows the image to be exported to a file.

Data Files
==========

The most convenient way to load data is to open the configuration file
(``.conf``) that is provided to the bundle adjustment tool. This file specifies
the locations of all relevant data and outputs, including camera KRTD files,
imagery, feature tracks and landmarks. It is also possible to load individual
images, cameras (via their KRTD files), track files, and landmark files. (Using
the feature detection/tracking configuration file is also supported; this
typically only provides images and, if already computed, feature tracks.)

.. notice::
  When loading cameras or images individually, cameras and images are
  associated in a first-loaded, first-matched manner. There is no way to load
  individual camera and image files that allows for cameras without images, or
  images without cameras, except at the end of the frame sequence. Similarly,
  frame identifiers are assigned sequentially based on the order in which files
  are loaded. In order for feature points to be correctly associated with their
  corresponding frames, the camera/image files must be loaded so that these
  automatically assigned identifies match those that were assigned by the
  feature detection/tracking pipeline.


Menu
====

File Menu
---------

:icon:`open` Open
  Presents a dialog that allows the selection of one or more data files to be
  loaded into the session.

:icon:`blank` Export
  Provides options for exporting various data.

:icon:`quit` Quit
  Exits the application.

Compute Menu
------------

:icon:`blank` Refine
  Applies bundle adjustment to the cameras and landmarks in order to refine the
  quality of the 3D reconstruction.

:icon:`blank` Align
  Applies a similarity transformation to the camera and landmark data so that
  the data has a standard ("canonical") alignment. Particularly, this attempts
  to orient the data so that the ground plane is parallel with the :f:`z = 0`
  plane (with the cameras in the :f:`+Z` direction). Additionally, the
  landmarks will be centered about the origin and scaled to an approximate
  variance of :f:`1.0`.

:icon:`blank` Reverse (Necker)
  Transforms the cameras and landmarks in a manner intended to break the
  refinement process out of a degenerate optimization (which can occur due to
  the Necker cube phenomena\ [#nc]_), by computing a best fit plane to the
  landmarks, mirroring the landmarks about said plane, and rotating the cameras
  180\ |deg| about their respective optical axes and 180\ |deg| about the
  best fit plane normal where each camera's optical axis intersects said plane.

View Menu
---------

:icon:`playback-play` Play Slideshow
  Toggles playback of the slideshow.

:icon:`playback-loop` Loop Slideshow
  Toggles if the slideshow should restart from the beginning after the last
  camera. When disabled, the slideshow ends when the last camera becomes
  active.

:icon:`blank` Match Matrix
  Opens a new `Match Matrix View`_.

Help Menu
---------

:icon:`help-manual` MapGUI User Manual
  Displays the user manual (i.e. this document) in the default web browser.

:icon:`mapgui` About MapGUI
  Shows copyright and version information about the application.

Glossary
========

Camera:
  A camera in MAP-Tk refers primarily to the model which describes the
  properties of a camera, including attributes such as focal length and world
  position and orientation. In the GUI, cameras are represented as frustums.

Feature:
  A feature is a location that corresponds to an "interesting" point, such as
  the corner of an object or other "notable" point. The term "feature points"
  typically refers to features detected in imagery.

Track:
  A track is a collection of correlated features; that is, detected feature
  points estimated to correspond to the same landmark.

Landmark:
  A landmark is an estimated world location of a "true" feature that is
  computed from a feature track.

Residual:
  A residual, in general, is the difference between an observed value and an
  estimated value\ [#er]_. In MAP-Tk, the observed value is typically a
  detected feature point, and the estimated value is a landmark.

.. [#nc] https://en.wikipedia.org/wiki/Necker_cube
.. [#er] https://en.wikipedia.org/wiki/Errors_and_residuals_in_statistics

.. |--|  unicode:: U+02014 .. em dash
.. |deg| unicode:: U+000B0 .. degree sign
