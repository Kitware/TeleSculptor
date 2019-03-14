===============================================================================
  TeleSculptor
===============================================================================

.. role:: f
   :class: math

.. contents::

Overview
========

The TeleSculptor graphical applicaton provides a means of computing
structure-from-motion and visualizing the results, as well as loading results
from the MAP-Tk command line tools. Entities available for display include the
images or video frames, tracked feature points, cameras frustums, 3D landmark
points, dense depth maps, and surfaces extracted volumetric data.

The GUI consists of a primary (world) view, secondary (camera and depth) views,
and a frame selection panel. The world view is a three dimensional view that
shows the cameras and landmarks in the computed world coordinate system. The
camera view shows the imagery from a single camera along with corresponding
feature points, projected landmarks, and estimation residuals. The depth map
view shows an estimated depth image for the current frame, if one is available.
The frame selection panel provides controls to select the active camera and to
use the slideshow mode. The camera view, depth view, and camera selection panel,
are dockable and can be rearranged or detached to suit user preference.

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

Additionally, the world view can display a representation of the ground plane
(i.e. the world coordinate plane :f:`z = 0`), and can display the frame image
for the active camera projected to that plane.  These projections result in
a stablized view of the video if the scene is largely planar and that plane
has been algined with `z = 0`.  There is a tool avaialable to reorient the
data for this purpose.

The world view also supports visualization of the depth image for the current
frame as either a dense RGB point cloud or a surface mesh.  It also can render
a 3D surface mesh extracted from the level set of a volumetric array.
The volume is typically the result of the fusion of several depth maps
using a truncated signed distance function or similar operator.
Computation of depth maps and volumetric data are not yet supported in the
TeleSculptor application.  These types of data must be computed by command
line tools and loaded into TeleSculptor for visualization by setting variables
in a configuration file.

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

:icon:`location` Edit Ground Control Points
  Toggles editing of ground control points.
  See `Editing Ground Control Points`_ for details.

:icon:`grid` Show Ground Plane Grid
  Toggles visibility of the ground plane. The ground plane is the :f:`z = 0`
  plane in world coordinates. The grid is centered about :f:`x = y = 0`,
  however the grid lines are otherwise strictly aesthetic and do not correspond
  to any particular values.

:icon:`roi` Show/Edit Region of Interest
  Toggles visibility of the region of interest selection in the world view.
  While visible, the ROI may be resized by clicking and dragging on any of the
  six handles on the faces of the ROI box.

:icon:`blank` Reset Region of Interest
  Resets the region of interest to the axis-aligned bounds of the entire
  dataset. This action is available via the
  :action:`roi Show/Edit Region of Interest` button's associated pop-up menu.

:icon:`depthmap` Show 3D Depth Map
  Toggles visibility of the depth map (if avaialble) rendered as a 3D point
  cloud or mesh; see `3D Depth Map Options`_.

:icon:`volume` Show Surface from Volume
  Toggles the visibility of the surface mesh extracted from volumetric data.
  This option is disabled if no volume data is loaded; see
  `Volume Surface Options`_.

:icon:`ruler` Enable Measurement Tool
  Toggles placing or editing of the ruler measurement tool. Initially |--| when
  the ruler has not yet been placed, or after it has been removed using
  :action:`- Reset Measurement Tool` |--| a ruler can be placed by clicking two
  points in the view. The depth of the points is calculated based on landmarks
  in the immediate vicinity of the point being placed, or the ground plane if
  no nearby landmarks are found. Once placed, the ruler's points may be moved
  freely. Placement of the ruler may be canceled by pressing the **Esc** key
  before placing the second point.

:icon:`blank` Reset Measurement Tool
  Removes the currently placed ruler. This action is available via the
  :action:`ruler Enable Measurement Tool` button's associated pop-up menu.

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

Point Options
-------------

The :action:`landmark Show Landmarks` pop-up provides additional controls that
can be used to control the display of the landmarks in the world view. (The
same controls are also used in the camera view to manipulate the display of
feature points and landmarks in that view.) These allow the color of the
items to be changed, as well as their size. Feature items (that is, feature
points and landmarks) are displayed as dots, with a fixed size-on-screen that
is independent of the view.

Several options for color are provided. The simplest is "solid color", which
displays all landmarks in the same, user selected color. "True color" displays
landmarks in the color estimated to correspond to the actual color of the point
in the real world scene, as computed from the input imagery. "Color by data"
uses color to visualize other per-point data, such as the number of individual
frames that contributed to ("observed") each landmark.

In addition to coloring by data, points may be filtered (selectively displayed)
according to their respective values of the currently selected data set.
Filtering may exclude points above or below selected lower or upper, or that
are not within said bounds.

3D Depth Map Options
--------------------

The :action:`depthmap Show Depth Map` pop-up provides additional controls on the
display of depth maps in the world view.  The options allow the depth map to be
rendered either as a 3D point cloud (one point per pixel) or a dense triangular
mesh (one vertex per pixel).  In either case, the rendered depth data is
colored by the RGB color values of the corresponding video frame.  A filter
option is also available to remove depth points based on thresholds on various
attriutes.  Currently these attributes are the Uniqueness Ratio and Best Cost
Value.  Images of these attibutes as well as the depth map itself are also
shown in the Depth Map View and the filter options selected here apply to that
view as well.  See `Depth Map View`_.

Volume Surface Options
----------------------

The :action:`volume Volume from Surface` pop-up provides additional controls on the
extraction and coloring of a surface from volumetric data.  The "Surface
threshold" parameter controls the value of the iso-surface at which the surface
is extracted from the volume.  The "Colorize surface" option, if checked,
allows coloring each vertex of the mesh.  The "Current frame" mode projects the
RGB values from the current frame onto the mesh, while the "All frames"
mode combines apperance projected from all frames or a subset of frame
sampled at a regular interval.  The "Color display" options determine how to
color the surface.  Options include mean color, median color, surface normal,
and number of observations.

Editing Ground Control Points
-----------------------------

The :action:`location Edit Ground Control Points` action allows the user to
enter or leave edit mode for ground control points. When not in edit mode,
the scene location of ground control points is fixed and cannot be changed,
nor can ground control points be selected in the world or camera views.

In edit mode, clicking on a ground control point in either view selects the
point in both views as well as the `Ground Control Points`_ panel. (Selecting
a point in the panel also selects it in both views.) Points may be dragged in
either view to change their scene location. Holding the **Shift** key while
moving constrains movement to one of the principle axes.

New points may be added by holding the **Ctrl** key while clicking. When
placing new ground control points in the view, TeleSculptor projects a ray into
the scene that corresponds to the location that was clicked and selects a
location along this ray based on landmarks in the immediate vicinity. If no
nearby landmark points are found, the new point is placed on the ground plane.

Pressing the **Del** key while in edit mode when one of the views has keyboard
focus will delete the currently selected ground control point.

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

Depth Map View
==============

The Depth Map View provides an image viewer similar to the Camera View but
specialized to display depth map images.  Depth map images are loaded from
VTK image (.vti) files associated with a particular video frame.  Often
there are only depth maps on a subset of frames.  The active (or most recent)
depth maps is displayed in this view by mapping depth to color.
The Depth Map View can also display image of other attributes associated
with the depth map such as the image color.  Some attributes like uniqueness
and best cost are associated with the algorithms used to generate the depth.
The same depth maps can be rendered in the World View as a point cloud.
Furthermore, depth map filtering options in the World View also apply to the
image rendering of the depth map in the Depth Map View.
Depth maps are currently not produced by MAP-Tk but require third-party
software to take MAP-Tk cameras and images to produce them.

Tool Bar
--------

:icon:`view-reset` Reset View
  Resets the view to the camera image extents.

:icon:`blank` Display mode
  Selects which image mode to display in the in the view: Color, Depth,
  Best Cost Value, Uniqueness Ratio; see `Color Map Options`_.
  The depth filters apply regardless of which image is shown.

Color Map Options
-----------------

In addition to selecting the mode under `Display Mode` there is also an
option to select the color mapping function for each mode except Color.
The mapping function describes how the scalar data field (e.g. depth) is
mapped to color.  Below the color map option are the minimum and maximum values
from the data used in the mapping.  The `Auto` checkbox, which is checked by
default, indicates that the values are determined automatically from the range
of values in the image data.  By unselected the `Auto` checkbox the minimum
and maximum values of the range can be adjusted manually for finer control of
the visualization.

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

Metadata
========

The metadata panel displays the collection of video metadata for the current
frame, if available. The set of fields is selected from the entire data set;
individual frames may be missing some or all fields.

Ground Control Points
=====================

The ground control points panel displays a list of all ground control points in
the current data set, as well as detailed information for the selected point.
Points have an automatically assigned ID (which may change between sessions)
and an optional user-provided name, which may be assigned or changed by editing
that column of the point (by double-clicking or pressing the edit key |--|
usually **F2**).

When a point is selected, changing its geodetic location (as described by the
latitude, longitude, and elevation text fields) automatically promotes the
point to a "user registered" point. These are points for which the geodetic
location has been externally measured and is therefore known to be correct.
The geodetic location of points which are not user registered is computed from
their scene location and the computed scene to geodetic transformation (if
available). User registered points are indicated by an icon in the ground
control point list.

Selecting a point in the list will select the same point in the world and
camera views if ground control point editing is active. Similarly, selecting a
ground control point in either view will select the same point in the list.
Note that moving a user registered point in the world or camera views (that is,
changing its scene location) does not change its geodetic location.

Tool Bar
--------

:icon:`copy-location` Copy Location
  Copies the geodetic location of the selected point to the clipboard. Several
  options of ordering and whether or not to include the elevation are provided.

:icon:`reset` Revert Changes
  Reverts user changes to the active ground control point's geodetic location,
  such that the point is no longer "user registered". This has no effect on
  points that are not user registered. Note also that the geodetic location
  will not change if a scene to geodetic transformation is not available.

:icon:`delete` Delete Point
  Deletes the active ground control point.

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

:icon:`blank` New Project
  Select a working directory for a project.  A project directory must be set
  before the tools in the Compute menu can be run.  These tool will write files
  into the project working directory.  A configuration file with the same name
  as the directory is also created in the directory.  The project configuration
  file stores references to the project data such as the source video and
  computed results like cameras, tracks, or landmarks that will be loaded back
  in when a project is opened.

:icon:`open` Open
  Presents a dialog that allows the selection of one or more data files to be
  loaded into the session.  Open is used to open a project config file, but can
  also be used to open other files for inspection, like cameras and videos.
  Once a project is created, this is how you open a video to be process.

:icon:`blank` Export
  Provides options for exporting various data.

:icon:`quit` Quit
  Exits the application.

Compute Menu
------------

:icon:`blank` Track Features
  Run feature tracking on the loaded video starting from the current frame.
  Features and descriptors are detected and each frame and cached into a file
  in the project directory.  Features are then matched between adjacent frames
  as well as between the current frame as past keyframes.  These feature
  matches form "tracks" through time, and each track has the potential to
  become a landmark.

:icon:`blank` Estimate Cameras/Landmarks
  Estimates cameras and landmarks starting with tracks and metadata.  This also
  runs bundle adjustment (refinement) along the way.  The goal is to
  incrementally add cameras and landmarks, while optimizing, to build up
  a consistent solution.

:icon:`blank` Save Frames
  Iterate through a video and save every frame as an image file in a
  subdirectory of the project directory.  This is needed when exporting
  the data to other tools that do not support video files.  This option
  must be run before importing a project into SketchUp.

:icon:`blank` Batch Compute Depth Maps
  Estimates several dense depth maps and corresponding point clouds on several
  frames spaced throughout the video.  This requires valid cameras and
  computes the results in the active ROI.  The algorithm run on each frame
  is the same as "Compute Single Depth Map", but intermediate solutions of
  each depth map are not rendered.

:icon:`blank` Fuse Depth Maps
  Fuse all computed depth maps into a single mesh surface using an integration
  volume specified by the ROI.  Note that this step requires an Nvidia GPU
  and may not be able to run of the ROI is too large for the GPU memory.

Compute Menu -> Advanced
------------------------

:icon:`blank` Filter Tracks
  Filter the tracks to retain a smaller subset of tracks that is still
  representative of the original set.  The intent is to make bundle adjustment
  (refine solution tool) faster without loosing critical constraints.  The
  filter attempts to remove the shortest tracks that span the same frames
  already covered by longer tracks.

:icon:`blank` Triangulate Landmarks
  For each available feature track, back project rays from the cameras that
  contain each track state and intersect those rays in 3D to estimate the
  location of a 3D landmark.  This requires both feature tracks and a
  reasonably accurate set of cameras.

:icon:`blank` Refine Solution
  Applies bundle adjustment to the cameras and landmarks in order to refine the
  quality of the 3D reconstruction. It aims to minimize this distance between
  the landmarks projected into each image by the cameras and the observed
  location of the corresponding feature tracks.

:icon:`blank` Reverse (Necker)
  Transforms the cameras and landmarks in a manner intended to break the
  refinement process out of a degenerate optimization (which can occur due to
  the Necker cube phenomena\ [#nc]_), by computing a best fit plane to the
  landmarks, mirroring the landmarks about said plane, and rotating the cameras
  180\ |deg| about their respective optical axes and 180\ |deg| about the
  best fit plane normal where each camera's optical axis intersects said plane.

:icon:`blank` Align
  Applies a similarity transformation to the camera and landmark data so that
  the data has a standard ("canonical") alignment. Particularly, this attempts
  to orient the data so that the ground plane is parallel with the :f:`z = 0`
  plane (with the cameras in the :f:`+Z` direction). Additionally, the
  landmarks will be centered about the origin and scaled to an approximate
  variance of :f:`1.0`.

:icon:`blank` Save Key Frames
  Iterate through a video and save every key frame as an image file in a
  subdirectory of the project directory.  Key frames are marked by the
  feature tracking algorithm.

:icon:`blank` Compute Single Depth Map
  Estimate a dense depth map and corresponding point cloud from the current
  frame.  This requires a valid camera on the current frame as well as cameras
  on other frames for triangulation.  It computes the solution within the
  active ROI and shows an incremental visualization of how the solution evolves.

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

:icon:`blank` Background Color
  Changes the background color of the world and camera views.

Help Menu
---------

:icon:`help-manual` TeleSculptor User Manual
  Displays the user manual (i.e. this document) in the default web browser.

:icon:`telesculptor` About TeleSculptor
  Shows copyright and version information about the application.

Glossary
========

Camera:
  A camera in TeleSculptor refers primarily to the model which describes the
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
  estimated value\ [#er]_. In TeleSculptor, the observed value is typically a
  detected feature point, and the estimated value is a landmark.

.. [#nc] https://en.wikipedia.org/wiki/Necker_cube
.. [#er] https://en.wikipedia.org/wiki/Errors_and_residuals_in_statistics

.. |--|  unicode:: U+02014 .. em dash
.. |deg| unicode:: U+000B0 .. degree sign
