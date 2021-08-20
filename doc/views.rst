.. _vuews:

.. |reset_view| image:: /../gui/icons/16x16/view-reset.png

.. |grid_button| image:: /../gui/icons/16x16/grid.png

======================
Views and Navigation
======================

The interface of TeleSculptor is made up of several viewing panes.  The primary pane is the 3D world view in the center that shows 3D reconstruction results and camera poses.  All
other view panes are optional and can be closed, rearranged, or popped out into new windows.  Closed panes can be reopened by selecting them under the *View* menu.  The size and
configuration of these panes is saved when the application closes and restored when opened again.  The secondary panes are

*	**Camera Selection** – shows a time slider, current frame number, and playback controls
*	**Camera View** – shows the current video frame and 2D geometry overlays
*	**Depth Map View** – shows the most recent depth map as a 2D image
*	**Metadata** – shows the metadata values associated to the current frame
*	**Ground Control Points** – shows a list of ground control points and their geodetic location
*	**Log Viewer** – shows the log output from running algorithms

The 3D world view has controls for navigating 3D space (e.g. pan, rotate, zoom) while 2D image views have controls for navigating 2D space (e.g. pan, zoom).  The controls for these
views are as follows

**3D Navigation**

*	**Rotate about scene center** – Left click and drag
*	**Rotate about camera axis** – Hold Ctrl and left click and drag
*	**Pan** – Middle click and drag or hold Shift and left click and drag
*	**Zoom** – Right click and drag or scroll wheel
*	**Select new scene center** – double left click on scene location

**2D Navigation**

*	**Pan** – Middle click and drag or hold Alt and left click and drag
*	**Zoom** – Right click and drag or scroll wheel

At any time, the Reset View button (|reset_view|) above the pane will reset view to show all the visible data.  The keyboard shortcut “R” has the same effect.  In some views the
drop-down menu under Reset View provides additional view options.  In the 3D world view you can set the camera to the primary coordinate viewing directions (Top, Left, Right, Front,
Back).  You can also toggle between using a perspective and orthographic view.  The 3D world view also has “Zoom to Landmarks” option under the Reset View button which is like Reset
View, but only considers landmarks.  This is a good way to focus on the landmarks or other 3D scene products while keeping the camera frustums visible.

A grid drawn in the 3D world view provides an additional frame of reference for spatial orientation.  You can toggle the grid visibility with the grid button (|grid_button|) above the
world view. This grid is always on a horizontal plane, initially Z=0.  The size of the grid is dynamic and adjusts to the scale of the scene data.  Grid cells do not represent real
units. The height of the plane also adjusts to match the height of estimated landmark points.  Additional coordinate axes can be enabled with the *World Axes* item in the *View*
menu.  The world axes scale to all visible data (including cameras) and the numbers and grid lines on the world axes do represent real world units.
