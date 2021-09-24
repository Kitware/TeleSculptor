.. _computeoptions:

================
Compute Options
================

.. image:: /images/compute_options.png
   :align: right
   :scale: 50 %

A few basic switches are available under the Compute->Options menu
to control the behavior of some of the algorithms.
Each of these options corresponds to a boolean
configuration value in the project file.
Checking these options will save that option in the project file.
In the future, we will provide a visual interface to configure many more
of the TeleSculptor algorithm options.
For now these basic options are presented in the menu:

Ignore Metadata
=================

When checked, this option causes the Initialize Cameras/Landmarks algorithm
to ignore any metadata that was loaded with the video.
This option is useful because it is often the case that the metadata is
incorrect and negatively impacts the algorithm rather than helping it.

Variable Lens
===============

By default, TeleSculptor assumes that all frames in a video are collected
with the same lens and zoom setting.
The intrinsic camera parameters are shared across the entire sequence.
This assumption gives the best results as long as the assumption holds true.
When the assumption does not hold, and the lens zooms or is changed
in the middle of the sequence, checking “Variable Lens” will instruct
TeleSculptor to estimate unique camera intrinsic parameters for each
video frame.

Fix Geo-Origin
================

TeleSculptor automatically selects a local origin near the centroid
of the data and records the geographic location of this origin point.
When the data is updated by running algorithms that origin point is
recalculated and may change.
In some cases, there are benefits to specifying the geographic origin to use
and keeping it fixed, for example, forcing two data sets to share
a common local origin for easier comparison.
Checking “Fix Geo-Origin” instructs TeleSculptor to keep the current
geographic origin and not recalculate a new one.

Use GPU
=========

Some TeleSculptor algorithms
(currently, depth map fusion)
are able to use CUDA to improve performance.
However, this requires an NVIDIA GPU
with sufficient GPU RAM.
Disabling this option instructs TeleSculptor
to rely solely on the CPU.
This is slower, but should give the same results
and may be used if a suitable GPU is not available.
