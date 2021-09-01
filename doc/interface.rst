.. _interface:

===============================================================================
User Interface
===============================================================================

.. role:: f
   :class: math

Overview
========

.. TODO merge this with index.rst

    The TeleSculptor graphical application
    provides a means of computing 3D models from video,
    especially aerial video.
    It includes tools for computing structure-from-motion
    to recover camera trajectory and internal camera parameters
    while estimating 3D scene points.
    It also has tools for multi-view stereo estimation of dense depth maps
    and tools for fusion of multiple depth maps into a full 3D surface.
    TeleSculptor also provides interactive visualization
    of these various products, including tools
    for editing |gcp|\ s
    and measuring 3D distances.
    Entities available for display include
    the video frames (or image sequence),
    tracked 2D feature points,
    cameras frustums,
    3D landmark points,
    dense depth maps,
    and surfaces extracted from the volumetric fusion of depth map data.

The GUI consists of
a primary (world) view,
secondary (camera and depth) views,
a frame selection panel,
and various other ancillary views and panels.
The world view is a three dimensional view
that shows the reconstruction results and camera poses
in the computed world coordinate system.
All other views and information panes are dockable,
and may be rearranged, resized, detached or closed (hidden)
according to user preference.
Closed panes may be reopened via the :menu:`View` menu,
or using the context pop-up menu
of the main menu bar or title bar of any pane.

The secondary panes are:

Camera Selection:
  Provides controls to view and select the active frame,
  and to cycle through the same in a manner
  similar to video playback or a slide show.

Camera View:
  Shows the imagery from the current frame,
  along with various two dimensional geometry.
  If a camera model is available,
  also shows three dimensional geometry
  mapped into the camera's point of view,
  as well as estimation residuals.

Depth Map View:
  Shows the most recent estimated depth map as a two dimensional image.

Metadata:
  Shows the metadata values associated with
  the current frame image.

Ground Control Points:
  Shows a list of |gcp|\ s and/or |crp|\ s,
  as well as details (including geodetic location) of the selected point.
  Also contains controls for editing or deleting points.

Log Viewer:
  Shows the log output for the application,
  particularly from running algorithms.

World View
==========

The world view provides a three dimensional view
showing all three dimensional products
in a common coordinate system.
It is especially useful for viewing the spatial relationships
between cameras and landmarks,
|gcp|\ s,
dense point clouds,
or surface meshes.

Landmarks are displayed as points,
while cameras are displayed as frustums of semi-arbitrary length,
with the active camera highlighted (see `Camera Options`_).
The "camera path",
which is a polyline connecting the camera centers,
can also be displayed.
Ground control points are displayed as "jacks";
three short lines meeting at right angles in 3D.

.. TODO formatting for next two paragraphs

Additionally, the world view can display
a representation of the ground plane
(i.e. the local coordinate plane :f:`z = 0`),
and can display the image for the active frame
projected to that plane.
These projections result in a stabilized view of the video
if the scene is largely planar
and that plane has been aligned with :f:`z = 0`.
Products from video with geospatial metadata
are automatically translated to the :f:`z = 0` plane.
For data without geospatial metadata,
the Estimate Cameras/Landmarks tool
attempts to fit a plane to the landmarks
as it estimates them to reorient the landmarks to :f:`z = 0`.

The world view also supports
visualization of the depth image for the current frame
as either a dense RGB point cloud or a surface mesh.
It also can render a three dimensional surface mesh;
either a "volume mesh",
extracted from the level set of a volumetric array,
or an externally provided mesh.
The volume is typically the result
of the fusion of several depth maps
using a truncated signed distance function or similar operator.

Navigation
----------

To rotate the view, press the |lmb| and drag.
By default, the view will rotate about the view's "look at" point,
which is initially, or after resetting the view,
at or near the center of the scene.
Holding :shortcut:`Ctrl` will rotate around the view axis
(i.e. will "roll" the view).
To pan, hold the |mmb| while dragging.
To zoom, hold the |rmb| and drag,
or rotate the mouse wheel.
A double-click will set a new "look at" point,
centering the view on that point and changing the rotation center.

Tool Bar
--------

:icon:`view-reset` Reset View
  Resets the view extents so that the entire scene is visible.
  The keyboard shortcut :shortcut:`R` provides the same effect.
  Additional actions are available
  via the action's associated pop-up.

:icon:`blank` Zoom to Landmarks
  Resets the view extents so that all landmarks are visible.
  This action is available via
  the |button-reset-view|'s associated pop-up menu.
  The keyboard shortcut is :shortcut:`Shift+R`.

:icon:`blank` View to World Top/Left/Right/Front/Back
  Resets the view rotation to a "standard" rotation,
  such that the view axes are parallel with the world axes.
  These actions are available via
  the |button-reset-view|'s associated pop-up menu.

:icon:`blank` Perspective
  Toggles the world view between perspective and parallel projection.
  Perspective projection more closely models human vision
  and is often useful for visualizing depth in the scene.
  Parallel projection can be useful for viewing the scene in profile.
  This action is available via
  the |button-reset-view|'s associated pop-up menu.
  The keyboard shortcut is :shortcut:`P`.

:icon:`image` Show Camera Frame Image
  Toggles visibility of the frame image
  projected onto the ground plane
  (if an associated camera model is available).
  The associated pop-up allows the opacity of the same to be adjusted.

:icon:`camera` Show Cameras
  Toggles visibility of cameras and related visualizations.
  The associated pop-up provides additional options;
  see `Camera Options`_.

:icon:`landmark` Show Landmarks
  Toggles visibility of landmarks.
  The associated pop-up provides additional options;
  see `Point Options`_.

:icon:`grid` Show Ground Plane Grid
  Toggles visibility of the ground plane grid.
  The ground plane is the :f:`z= 0` plane in local 3D coordinates.
  The grid is centered about :f:`x = y = 0`,
  however the grid lines are otherwise strictly aesthetic
  and do not correspond to any particular values.
  The size of the grid adjusts to fit the scene.

:icon:`roi` Show/Edit Region of Interest
  Toggles visibility of the region of interest selection in the world view.
  While visible, the ROI may be resized by clicking and dragging
  on any of the six handles on the faces of the ROI box.

:icon:`blank` Reset Region of Interest
  Resets the region of interest to the axis-aligned box
  containing 80% of the landmark points
  plus an additional 50% padding.
  This action is available via
  the |button-roi|'s associated pop-up menu.

:icon:`depthmap` Show 3D Depth Map
  Toggles visibility of the depth map (if available) rendered as a 3D point
  cloud or mesh; see `3D Depth Map Options`_.

:icon:`volume` Show Surface from Volume
  Toggles the visibility of the surface mesh extracted from volumetric data;
  see `Volume Surface Options`_.

.. TODO fix following ref when content is moved

:icon:`location` Edit Ground Control Points
  Toggles editing of |gcp|\ s.
  See `Editing Ground Control Points`_ for details.
  The keyboard shortcut is :shortcut:`Ctrl+G`.

:icon:`ruler` Enable Measurement Tool
  Toggles placing or editing of the ruler measurement tool.
  Initially |--| when the ruler has not yet been placed,
  or after it has been removed using :action:`- Reset Measurement Tool`
  |--| a ruler can be placed by clicking two points in the view.
  The depth of the points is calculated based on
  landmarks, depth map points, or the surface mesh
  location under the mouse cursor.
  Turn off visibility of objects to avoid selecting them.
  The ground plane is selected if no nearby geometry is found.
  Once placed, the ruler's points may be moved freely.
  Holding down the :shortcut:`z` key
  constrains the ruler to vertical measurements.
  Holding down the :shortcut:`x` or :shortcut:`y` keys
  constraints the ruler to a horizontal plane.
  Placement of the ruler may be canceled
  by pressing the :shortcut:`Esc` key
  before placing the second point.

:icon:`blank` Reset Ruler
  Removes the currently placed ruler.
  This action is available
  via the |button-measure|'s associated pop-up menu.

Camera Options
--------------

The :action:`camera Show Cameras` pop-up provides additional controls
that can be used to control the display of the cameras in the world view.
These allow changing the color of
both the active and inactive cameras
as well as the camera path,
changing the size of the camera frustums,
and toggling visibility of the inactive cameras and camera path
separate from the overall camera visibility.

The camera scale controls are relative to a "base size"
that is computed from the extents of the scene data.
The inactive camera scale is relative to the active camera scale,
with the maximum allowed value
giving active and inactive camera frustums the same size.

Point Options
-------------

The :action:`landmark Show Landmarks` pop-up provides additional controls
that can be used to control the display of the landmarks in the world view.
(The same controls are also used in the camera view
to manipulate the display of feature points and landmarks in that view.)
These allow the color of the items to be changed, as well as their size.
Feature items (that is, feature points and landmarks) are displayed as dots,
with a fixed size-on-screen that is independent of the view.

Several options for color are provided.
The simplest is "solid color",
which displays all landmarks in the same, user selected color.
"True color" displays landmarks in the color
estimated to correspond to the actual color
of the point in the real world scene,
as computed from the input imagery.
"Color by data" uses color to visualize other per-point data,
such as the number of individual frames
that contributed to ("observed") each landmark.

In addition to coloring by data,
points may be filtered (selectively displayed)
according to their respective values of the currently selected data set.
Filtering may exclude points above or below selected lower or upper bounds,
or that are not within said bounds.

3D Depth Map Options
--------------------

The :action:`depthmap Show 3D Depth Map` pop-up
provides additional controls
on the display of depth maps in the world view.
The options allow the depth map to be rendered
either as a 3D point cloud (one point per pixel)
or a dense triangular mesh (one vertex per pixel).
In either case, the rendered depth data is colored
by the RGB color values of the corresponding video frame.
A filter option is also available to remove depth points
based on thresholds on various attributes.
Currently, these attributes are the Weight and Uncertainty.
Images of these attributes as well as the depth map itself
are also shown in the Depth Map View
and the filter options selected here
apply to that view as well.
See `Depth Map View`_.

Volume Surface Options
----------------------

The :action:`volume Show Surface from Volume` pop-up
provides additional controls
on the extraction and coloring
of a surface from volumetric data.
The "Surface threshold" parameter
controls the value of the isosurface
at which the surface is extracted from the volume.
The "Colorize surface" option, if checked,
allows coloring each vertex of the mesh.
The "Current frame" mode projects the RGB values
from the current frame onto the mesh,
while the "All frames" mode
combines appearance projected from all frames
or a subset of frame sampled at a regular interval.
The "Color display" options determine how to color the surface.
Options include
mean color,
median color,
surface normal,
and number of observations.

.. TODO move following to a different page with also CRP documentation,
        and probably consolidate with cameracalibration.rst

Editing Ground Control Points
-----------------------------

The :action:`location Edit Ground Control Points` action
allows the user to enter or leave edit mode for |gcp|\ s.
When not in edit mode,
the scene location of |gcp|\ s is fixed and cannot be changed,
nor can |gcp|\ s be selected in the world or camera views.

In edit mode, clicking on a |gcp| in either view
selects the point in both views
as well as the `Ground Control Points`_ panel.
(Selecting a point in the panel also selects it in both views.)
Points may be dragged in either view to change their scene location.
Holding the :shortcut:`Shift` key while moving
constrains movement to one of the principle axes.

New points may be added
by holding the :shortcut:`Ctrl` key while clicking.
When placing new |gcp|\ s in the view,
TeleSculptor projects a ray into the scene
that corresponds to the location that was clicked
and selects a location along this ray
based on scene elements in the immediate vicinity.
If no nearby landmark points are found,
the new point is placed on the ground plane.

Pressing the :shortcut:`Del` key while in edit mode
when one of the views has keyboard focus
will delete the currently selected |gcp|.

Camera View
===========

The camera view provides a camera space view
of detected feature points, computed landmarks
and |gcp|\ s (all projected to the camera space),
|crp|\ s,
and the corresponding input imagery,
for the active frame.
Additionally, the estimation residuals
|--| the difference between landmarks
and feature points which participated in computing their estimated positions
|--| can be displayed as line segments between the feature point location and
projected landmark location.

Navigation
----------

To pan the view, hold the |mmb| and drag,
or drag while holding :shortcut:`Alt` and the |lmb|.
To zoom, hold the |rmb| and drag,
or rotate the mouse wheel.

Tool Bar
--------

:icon:`view-reset` Reset View
  Resets the view to the frame image extents.
  Additional actions are available
  via the action's associated pop-up.

:icon:`blank` Zoom Extents
  Resets the view extents so that the entire scene is visible.
  This action is available via
  the |button-reset-view|'s associated pop-up menu.

:icon:`image` Show Camera Frame Image
  Toggles visibility of the frame image.
  The associated pop-up
  allows the opacity of the same to be adjusted.

:icon:`feature` Show Feature Points
  Toggles visibility of feature points / trails.
  The associated pop-up provides additional options;
  see `Feature Options`_.

:icon:`landmark` Show Landmarks
  Toggles visibility of landmarks.
  The associated pop-up provides additional options;
  see `Point Options`_.

:icon:`residual` Show Residuals
  Toggles visibility of the landmark estimation residuals.
  The associated pop-up
  allows the color of the displayed residuals to be changed.

.. TODO fix following ref when content is moved

:icon:`location` Edit Camera Registration Points
  Toggles editing of |crp|\ s.
  See :doc:`cameracalibration` for details.
  The associated pop-up
  allows computing a camera model
  on the current frame using |crp|\ s.
  The keyboard shortcut is :shortcut:`Ctrl+R`.

Feature Options
---------------

In addition to active feature points,
which have all the options described in `Point Options`_,
the position of feature points on adjacent frames
may also be displayed by enabling :action:`- Trails`.
For image collections where frames
adjacent in the frame list
also have spatially similar camera poses
(especially when using consecutive video frames as input),
these may be useful as an additional means of visualizing camera motion.

The trail color and length
(number of adjacent frames to be used)
may be changed,
as well as whether to show trails
only for lower-numbered frames ("historic" mode),
or for all adjacent frames ("symmetric" mode).
In all cases, trails are displayed
only for active feature points.

Depth Map View
==============

The Depth Map View provides an image viewer
similar to the Camera View
but specialized to display depth map images.
Depth map images are loaded from VTK image (:path:`.vti`) files
associated with a particular video frame.
Often there are only depth maps on a subset of frames.
The active (or most recent) depth map
is displayed in this view by mapping depth to color.
The Depth Map View can also display an image representation
of other attributes associated with the depth map,
such as the image color.
Some attributes like uniqueness and best cost
are associated with the algorithms used to generate the depth values.
The same depth maps can be rendered in the World View as a point cloud.
Furthermore, depth map filtering options in the World View
also apply to the image rendering of the depth map in the Depth Map View.

Navigation functions the same as the `Camera View`_.

Tool Bar
--------

:icon:`view-reset` Reset View
  Resets the view to the frame image extents.

:icon:`blank` Display Mode
  Selects which image mode to display in the in the view: Color, Depth,
  Best Cost Value, Uniqueness Ratio; see `Color Map Options`_.
  The depth filters apply regardless of which image is shown.

Color Map Options
-----------------

In addition to selecting the mode under :action:`- Display Mode`,
there is also an option to select the color mapping function
for each mode except Color.
The mapping function describes
how the scalar data field (e.g. depth)
is mapped to color.
Below the color map option are the minimum and maximum values
from the data used in the mapping.
The :action:`- Auto` check box,
which is checked by default,
indicates that the values are determined automatically
from the range of values in the image data.
By unchecking the :action:`- Auto` checkbox,
the minimum and maximum values of the range
can be adjusted manually
for finer control of the visualization.

Camera Selection
================

The camera selection panel contains a large slider
used to select the active frame,
and used to control which frame's imagery and feature points
are displayed in the camera view.
If the camera pose for the frame is known,
the active camera is also highlighted in the world view,
A spin box next to the slider shows the active frame number,
and can also be used to select the active frame.
Note that the frame numbers need not be consecutive.
Some video readers are configured to only read every `N`-th frame,
where `N` may be 10, for example.
This helps cut down on data redundancy in video.
The frame sampling rate can be configured
by opening the project configuration file (:path:`.conf`)
in a text editor.

The controls to the right of the panel
control the application's slideshow mode.
Slideshow mode automatically increments
through the loaded frames at a fixed rate.
This can be used to view the feature points
for each frame image in sequence.
Setting the delay between cameras sufficiently low
can be used to simulate video playback
for image sequences taken from a motion imagery source.

The slideshow action controls
are also available via the :menu:`|menu-view|` menu.
The small slider controls the delay between slides.
The slider response is logarithmic,
with single steps in one-tenth powers of ten.
The slider tool tip includes the current delay
in human readable units.
Several frame filters are also available
in the :menu:`|menu-view|` menu.
These filters allow limiting the frames
to show to a specific subset,
such as key frames
or frames with tracking data.

Metadata
========

The metadata panel displays the collection of video metadata
for the current frame, if available.
The set of fields is selected from the entire data set;
individual frames may be missing some or all fields.
The metadata itself is provied by the video reader.
For encoded video files, TeleSculptor supports
key-length-value (KLV) encoding
following the |misb|_ (MISB) |misb-0104| and |misb-0601| standards.
Customized video readers can read metadata from other sources,
such as supplimentary text files or EXIF data.

Ground Control Points
=====================

The |gcp|\ s panel displays a list
of all |gcp|\ s and |crp|\ s in the current data set,
as well as detailed information for the selected point.
Points have an automatically assigned ID
(which may change between sessions)
and an optional user-provided name,
which may be assigned or changed
by editing that column of the point
(by double-clicking
or pressing the edit key
|--| usually :shortcut:`F2`).

Selecting a point in the list
will select the same point
in the world and camera views.
Similarly, selecting a |gcp| or |crp| in either view
(when the corresponding edit mode is active)
will select the same point in the list.

.. TODO move following to a different page with also CRP documentation

When a point is selected,
changing its geodetic location
(as described by the latitude, longitude, and elevation text fields)
automatically promotes the point to a "user registered" point.
These are points for which the geodetic location
has been externally measured and is therefore known to be correct.
The geodetic location of points which are not user registered
is computed from their scene location
and the computed scene to geodetic transformation (if available).
User registered points are indicated by a glyph (|glyph-surveyed|)
in the |gcp| list.

Note that moving a user registered point
in the world or camera views
(that is, changing its scene location)
does not change its geodetic location.

Tool Bar
--------

:icon:`copy-location` Copy Location
  Copies the geodetic location of the selected point to the clipboard.
  Several options for coordinate ordering
  and whether or not to include the elevation
  are provided.

:icon:`apply` Apply Similarity Transform
  Estimates a 3D similarity transformation
  to best align the |gcp| locations
  with the specified geodetic locations.
  At least three "user registered" |gcp|\ s are required.
  That is, at least three points
  must have manually specified latitude, longitude, and altitude.
  The estimated transform is applied to all data
  (cameras, landmarks, depth maps, etc.).

:icon:`reset` Revert Changes
  Reverts user changes to the active |gcp|'s geodetic location,
  such that the point is no longer "user registered".
  This has no effect on points that are not user registered.
  Note also that the geodetic location will not change
  if a scene to geodetic transformation is not available.

:icon:`delete` Delete Point
  Deletes the active |gcp|.

Match Matrix View
=================

The match matrix view provides a visualization
of the feature point associations across frames.
Pixels in the image correspond to values in the "match matrix"
representing the number of feature points
that feature detection has determined
correspond to the same real world feature.
Several options are provided to adjust the visualization:

* Layout controls the position of "identity" values,
  i.e. values that compare a frame to itself
  rather than a distinct frame.
  The default, "diagonal", simply maps the frame number
  directly to both the :f:`X` and :f:`Y` axes.
  "Horizontal" skews the image so that the :f:`y` values
  are relative to the "identity" values,
  placing them in a horizontal line at :f:`y = 0`,
  with positive :f:`y` representing "later" frames,
  and negative :f:`y` representing "earlier" frames.
  "Vertical" reverses these axes.

* Orientation controls which screen direction is considered positive :f:`Y`.
  The default, "matrix", uses down for positive :f:`Y`,
  as in textual value tables
  (e.g. textual listings of matrices, spreadsheets) or images.
  "Graph" uses up for positive :f:`Y`, as in most graphical plots.

* Values controls what values are used for each pixel.
  The default, "absolute",  uses the raw number of feature point correlations
  (which, for "identity" values
  is equal to the total number of feature points on that frame).
  "Relative (combined)" mode uses the percent of common feature points
  relative to the total number of distinct feature points
  on each frame being compared.
  The other two "relative" modes give the percent
  relative to the total number of feature points
  for the frame represented by either the :f:`X` or :f:`Y` axis.

* Scale controls the scaling function that is applied to the values
  produced according to the value mode.
  The choices are "linear", "logarithmic" and "exponential",
  and should be self explanatory.
  In absolute value mode,
  logarithmic scale uses the maximum value as the logarithm base.
  Otherwise, the base can be adjusted with the "range" control,
  which applies a pre-scale to the value before computing the logarithm
  (thereby allowing the shape of the scaling curve to be adjusted).
  Exponential scale allows the user to select the exponent.

* Color provides the set of colors to which scaled values are mapped.
  Several presets are available according to user taste.
  Different presets may help emphasize different aspects of the data.

Moving the mouse over the image
will display which frames are being compared
and the number or percentage of feature correlations
in the status bar.
The match matrix view also allows the image to be exported to a file.

.. TODO move following section elsewhere

Data Files
==========

TeleSculptor supports visualization of various data files
(landmarks, cameras, etc.) that are computed in other tools.
However the recommended workflow for most users
is to simply load a video and derive all other product from it.
Video files are loaded using `File` |rarrow| `Import` |rarrow| `Imagery...`.

Before computing any products from video,
a "Project" directory is needed to store the results.
A project is created with `File` |rarrow| `New Project`
which asks the user to provide a path to a working directory.
Inside this directory, a "Project File" is created
(name matching the directory name plus extension :path:`.conf`)
to store project settings.
Various other result files are also written to the project directory.
To open an existing project,
use `File` |rarrow| `Open Project...`
and navigate to an existing :path:`.conf` file.

.. note::
  When loading cameras or images individually,
  cameras and images are associated
  in a first-loaded, first-matched manner.
  There is no way to load individual camera and image files
  that allows for cameras without images,
  or images without cameras,
  except at the end of the frame sequence.
  Similarly, frame identifiers are assigned sequentially
  based on the order in which files are loaded.
  In order for feature points to be correctly associated
  with their corresponding frames,
  the camera/image files must be loaded
  so that these automatically assigned identifies
  match those that were assigned
  by the feature detection/tracking pipeline.

Menu
====

File Menu
---------

:icon:`blank` New Project
  Select a working directory for a project.
  A project directory must be set
  before the tools in the Compute menu can be run.
  These tool will write files into the project working directory.
  A configuration file with the same name as the directory
  is also created in the directory.
  The project configuration file
  stores references to the project data
  such as the source video
  and computed results like cameras, tracks, or landmarks
  that will be loaded back in when a project is opened.

:icon:`open` Open Project
  Select an existing project configuration.
  The project configuration will often include
  references to various data files
  which are frequently stored in the same directory
  as the project configuration.

:icon:`blank` Import
  Provides options for importing/loading various types of data
  into the current project.
  The user must select the type of data to be loaded,
  as some data files use the same file extension.

:icon:`blank` Export
  Provides options for exporting various data.

:icon:`quit` Quit
  Exits the application.

Compute Menu
------------

:icon:`blank` Run End-to-End
  Runs the entire processing pipeline from end-to-end.
  This tools runs Track Features,
  then Estimate Cameras/Landmarks,
  then Batch Compute Depth Maps,
  then Fuse Depth Maps.

:icon:`blank` Track Features
  Run feature tracking on the loaded video
  starting from the current frame.
  Features and descriptors are detected in each frame
  and cached into a file in the project directory.
  Features are then matched between adjacent frames
  as well as between the current frame as past keyframes.
  These feature matches form "tracks" through time,
  and each track has the potential to become a landmark.

:icon:`blank` Estimate Cameras/Landmarks
  Estimates cameras and landmarks starting with tracks and metadata.
  This also runs bundle adjustment (refinement) along the way.
  The goal is to incrementally add cameras and landmarks,
  while optimizing, to build up a consistent solution.

:icon:`blank` Save Frames
  Iterate through a video and save every frame
  as an image file in a subdirectory of the project directory.
  This is needed when exporting the data
  to other tools that do not support video files.
  This option must be run before importing a project into SketchUp.

:icon:`blank` Batch Compute Depth Maps
  Estimates several dense depth maps
  and corresponding point clouds
  on several frames spaced throughout the video.
  This requires valid cameras
  and computes the results in the active ROI.
  The algorithm run on each frame is the same as
  :menu:`Compute` |rarrow|
  :menu:`Advanced` |rarrow|
  :menu:`Compute Single Depth Map`,
  but intermediate solutions of each depth map are not rendered.

:icon:`blank` Fuse Depth Maps
  Fuse all computed depth maps into a single mesh surface
  using an integration volume specified by the ROI.
  Note that this step requires an NVIDIA GPU
  and may not be able to run
  if the ROI is too large for the GPU memory.

:icon:`blank` Advanced
  Provides access to additional, lower level tools.
  See :doc:`advancedtools` for details.

.. TODO consolidate following with advancedtools.rst

Compute Menu |rarrow| Advanced
------------------------------

:icon:`blank` Filter Tracks
  Filter the tracks to retain a smaller subset of tracks
  that is still representative of the original set.
  The intent is to make bundle adjustment (:action:`- Refine Solution`)
  faster without loosing critical constraints.
  The filter attempts to remove the shortest tracks
  that span the same frames already covered by longer tracks.

:icon:`blank` Triangulate Landmarks
  For each available feature track,
  back project rays from the cameras that contain each track state
  and intersect those rays in 3D to estimate the location of a 3D landmark.
  This requires both feature tracks and a reasonably accurate set of cameras.

:icon:`blank` Refine Solution
  Applies bundle adjustment to the cameras and landmarks
  in order to refine the quality of the 3D reconstruction.
  It aims to minimize this distance
  between the landmarks projected into each image by the cameras
  and the observed location of the corresponding feature tracks.

:icon:`blank` Reverse (Necker)
  Transforms the cameras and landmarks
  in a manner intended to break the refinement process
  out of a degenerate optimization
  (which can occur due to the Necker cube phenomena\ [#nc]_),
  by computing a best fit plane to the landmarks,
  mirroring the landmarks about said plane,
  and rotating the cameras 180 |deg|
  about their respective optical axes
  and 180 |deg| about the best fit plane normal
  where each camera's optical axis intersects said plane.

:icon:`blank` Align
  Applies a similarity transformation to the camera and landmark data
  so that the data has a standard ("canonical") alignment.
  Particularly, this attempts to orient the data
  so that the ground plane is parallel with the :f:`z = 0` plane
  (with the cameras in the :f:`+Z` direction).
  Additionally, the landmarks will be centered about the origin
  and scaled to an approximate variance of :f:`1.0`.

:icon:`blank` Save Key Frames
  Iterate through a video and save every key frame as an image file
  in a subdirectory of the project directory.
  Key frames are marked by the feature tracking algorithm.

:icon:`blank` Compute Single Depth Map
  Estimate a dense depth map
  and corresponding point cloud
  for the current frame.
  This requires a valid camera on the current frame
  as well as cameras on other frames for triangulation.
  It computes the solution within the active ROI
  and shows an incremental visualization of how the solution evolves.

Compute Menu |rarrow| Options
-----------------------------

:icon:`blank` Ignore Metadata
  Ignore the metadata fields (e.g. from KLV)
  that are attached to the imagery.
  If this option is set, metadata will not be used
  in camera estimation or in geolocation.
  The main reason to use this option
  is when the metadata is known to be invalid.

:icon:`blank` Variable Lens
  This option allows estimation of camera models
  in which the lens model (e.g. focal length) can change over time.
  If this option is off, all cameras will share the same lens model.
  If the lens does not change between frames,
  it is best to use a single model.

:icon:`blank` Fix Geo-Origin
  If checked, this option will prevent TeleSculptor
  from resetting the geospatial origin to a point centered on the data.
  Normally, TeleSculptor automatically chooses an origin,
  but there are use cases where it is helpful
  to specify an origin and keep it fixed,
  such as comparing results across different data of the same location.

.. _view-menu:

View Menu
---------

:icon:`playback-play` Play Slideshow
  Toggles playback of the slideshow.

:icon:`playback-loop` Loop Slideshow
  Toggles if the slideshow should restart from the beginning
  after the last frame.
  When disabled, the slideshow ends
  when the last frame becomes active.

:icon:`blank` Match Matrix
  Opens a new `Match Matrix View`_.

:icon:`blank` Background Color
  Changes the background color of the world and camera views.

:icon:`blank` World Axes
  Toggles visibility of `X`, `Y`, and `Z` axes in the world view
  showing numerical values for distances at regular intervals on these axes.
  The size of these axes is set to span all visible scene objects,
  including the camera path.
  If the axes are too large,
  hiding scene objects like cameras and the ground plane
  will shrink the coverage to the remaining visible data.

:icon:`blank` Keyframes Only
  Limit frame numbers in the camera selection pane
  to allow only frames that were designated
  as "keyframes" by the feature tracker.
  The number of keyframes is typically very small.

:icon:`blank` Tracked Frames Only
  Limit frame numbers in the camera selection pane
  to allow only frames that contain feature tracking results.
  The feature tracker only processes a fixed number of frames
  (default: 500) distributed through the video.
  Enabling this option skips unprocessed frames during playback,
  which avoids flicker of the display
  that occurs when unprocessed frames are drawn.

:icon:`blank` Antialias Views
  Toggles use of an anti-aliasing filter
  in the world, camera and depth views.
  Anti-aliasing is accomplished
  via a post-processing filter (FXAA)
  that may produce undesirable artifacts.
  At this time, anti-aliasing via multi-sampling (MSAA)
  is not supported.

Help Menu
---------

:icon:`help-manual` TeleSculptor User Manual
  Displays this documentation
  in the default web browser.

:icon:`telesculptor` About TeleSculptor
  Shows copyright and version information about the application.

----

.. [#nc] https://en.wikipedia.org/wiki/Necker_cube

.. |lmb| replace:: primary (usually left) mouse button
.. |rmb| replace:: secondary (usually right) mouse button
.. |mmb| replace:: middle (wheel) mouse button

.. |misb| replace:: Motion Imagery Standards Board
.. |misb-0104| replace:: 0104
.. |misb-0601| replace:: 0601

.. _misb: https://gwg.nga.mil/misb/
.. _misb-0104: https://gwg.nga.mil/misb/docs/eg/EG0104.4.pdf
.. _misb-0601: https://gwg.nga.mil/misb/docs/standards/ST0601.6.pdf

.. |button-reset-view| replace:: :action:`view-reset Reset View` button
.. |button-roi| replace:: :action:`roi Show/Edit Region of Interest` button
.. |button-measure| replace:: :action:`ruler Enable Measurement Tool` button

.. |menu-view| replace:: :ref:`View <view-menu>`

.. |glyph-surveyed| image:: images/surveyed.svg
  :class: glyph
