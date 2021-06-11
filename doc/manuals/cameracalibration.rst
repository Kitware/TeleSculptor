.. _cameracalibration:

=========================
Manual Camera Calibration
=========================

This section describes the functions for manaul camera calibration capabilities in TeleSculptor (TS)
and steps to achieve, possibly correct, and export a camera calibration model.

Preparation
===========

TeleSculptor can work with ffmpeg-readable videos (e.g. mpg, mp4, avi) or with image sequences.
The latter can be specified by listing paths to individual images in a plain text file (e.g. images.txt)
using one line per path/to/image/file. Standard image formats (e.g. bmp, png, jpg) are supported.

TeleSculptor has a capability to import PLY mesh or point cloud files via File/Import/Mesh.
Point clouds are imported as faceless meshes.
The imported mesh is used as a 3D target for calibration. RGB[A] color space is currently supported.
The alpha channel can be used to set mesh transparency levels.

We recommend geo-registering the mesh before importing it to TS to have the output calibrated
camera parameters in mesh metric units and also automatically geo-registered with the 3D model.

Calibration
===========

Start the TeleSculptor GUI and proceed with the following steps in a new or an existing project.

.. |gcp_button| image:: /../gui/icons/22x22/location.png

New project
-----------

Choose menu: File/New Project… (Ctrl+N) and select a folder for the new project,
creating one if necessary. The project ini file name would have the same name by default.

Choose menu: File/Import/Imagery… (Ctrl+O,I) and select a video or an image sequence text file.
Large videos would imply a long search for metadata, which can be canceled via menu: Compute/Cancel.

Choose menu: File/Import/Mesh… (Ctrl+O,M) and select an PLY file with a mesh or a point cloud,
which should appear in the 3D viewer.

Scroll to the desired video/image frame using the Camera Selection slider
and observe it in the Camera View pane. Rotate the 3D model to the desired view,
e.g. resembling the current image.

To place/edit ground control points (GCP), click the 3D viewer GCP tool |gcp_button| 
to start GCP creation and editing.
Place six or more GCPs on the 3D model at the feature points visible from the camera view,
observe their IDs appear in the Ground Control Points list, name them, if desired.

To build 3D-2D point correspondences, click camera registration points (CRP) tool |gcp_button|
in the Camera View.
For each GCP in the list (also high-lighted in the 3D viewer):
select a 3D point, then click on the Camera View image, where its 2D projection is expected to be.
Observe the target-like icon in the GCP list for the 3D point having a corresponding 2D point.
Select another 3D GCP and create-click-move its 2D counterpart.
Continue with the 3D-2D correspondence process until the count of 6 or more.

In the Camera View, select the [Place/edit CRP] drop-down [Compute Camera] to invoke the calibration process.
Check the Log Viewer for the re-projection error or for any error messages, e.g. not enough calibration points.
Observe the 3D viewer for the image projection corresponding to the newly calibrated camera.
Click the tree-expandable marker in the GCP list next to a GCP point to see the frame numbers
this point was used in for camera calibration.
Double-Click on the desired frame number to jump to that frame.

Move 3D and/or 2D points to adjust the camera calibration as needed.
Zoom in and out, if necessary in both 3D and 2D views.
Scroll to a different frame and repeat the camera calibration process.

Choose menu: File/Export/KRTD Cameras… to output the parameters of all calibrated cameras to KRTD text files,
each capturing the intrinsic matrix K, Rotation matrix, Translation vector and lens Distortion parameters.

Choose menu: File/Export/Ground Control Points… to output all the GCPs
and their corresponding 2D CRPs for each frame 3D-2D correspondence were built.

Existing project
----------------
Choose, menu: File/Open Project… (Ctrl+O,P) to open an existing project,
its corresponding video/image sequence and the 3D mesh or point cloud,
which are loaded automatically.
Follow the calibration procedure described in the New project section.