.. _advancedconfig:

==============================
Advanced Configuration Options
==============================

Most users should not need these advanced features,
but some may be interested.

Changing Frame Sampling Rate
============================

By default, all video frames are loaded when opening a video
(though not all are processed in feature tracking).
If the number of frames is too large to manage
it is possible to read only every Nth frame
by changing a setting in the project configuration.
First, close the application.
Next, open the project :path:`.conf` file in a text editor like Notepad.
Look for the following line:

.. code-block:: bash

  video_reader:filter:output_nth_frame=1

Increase the number from 1 to 10 to sample every tenth frame, for example.
Save the :path:`.conf` file in the text editor
and open that file again in the TeleSculptor application.

Modifying Algorithm Parameters
==============================

TeleSculptor is a highly configurable application,
though most of the configuration options
are not yet exposed to the user interface.
Each tool in the compute menu calls an algorithm from the KWIVER toolkit
and each algorithm is configurable at run time.
Algorithms can even be swapped out for other algorithms at run time.
One can contribute a new algorithm without recompiling TeleSculptor
by dropping in a new :path:`.dll` and updating the configuration files.
All this configurability is managed with configuration files.
The project file is one example of configuration file,
but there are also many default configuration files
loaded by TeleSculptor at run time.
The default configuration files for a standard install path
are found in these two locations:

.. parsed-literal::
  :class: wrap

  C: |backslash| Program Files |backslash| TeleSculptor |version|
  |backslash| share |backslash| telesculptor |backslash| |version|
  |backslash| config

.. parsed-literal::
  :class: wrap

  C: |backslash| Program Files |backslash| TeleSculptor |version|
  |backslash| share |backslash| kwiver |backslash| |kwiver_version|
  |backslash| config

TeleSculptor specific configurations are found in the first directory
and these include configurations for KWIVER algorithms
found in the second directory.
It is recommended that you not modify these values,
but instead copy some of these files into your project directory
and modify the copies.
TeleSculptor will load configuration files from the project directory first.
Each of the tools in the *Compute* menu
loads a configuration file when it is run.
These have names starting with a :path:`gui\_` prefix.
For example:

.. table::
   :align: center

   +---------------------------+---------------------------------------+
   | Compute Menu Name         | Configuration File Entry Point        |
   +===========================+=======================================+
   | Track Features            | :path:`gui_track_features.conf`       |
   +---------------------------+---------------------------------------+
   | Estimate Camera/Landmarks | :path:`gui_initialize.conf`           |
   +---------------------------+---------------------------------------+
   | Save Frames               | :path:`gui_frame_image_writer.conf`   |
   +---------------------------+---------------------------------------+
   | Batch Compute Depth Maps  | :path:`gui_compute_depth.conf`        |
   +---------------------------+---------------------------------------+
   | Fuse Depth Maps           | :path:`gui_integrate_depth_maps.conf` |
   +---------------------------+---------------------------------------+

Most of these configuration files also
reference other configuration files with an ``include`` statement.
Configuration values for tools can also be added to the project file
but copying the GUI configuration files into the project directory
adds more flexibility because these files
are reloaded each time the tool is run,
which allows changing parameters between runs
without loading the project.

As an example, consider
changing the maximum number of frames to use in the feature tracker.
First, copy :path:`gui_track_features.conf` into your project directory.
Open this file and look for the following section:

.. code-block:: bash

   # Parameters for the feature tracker
   block feature_tracker
     include core_feature_tracker.conf

     # The maximum number of frames for the GUI to process.
     # The tracker will choose frames distributed over the video
     max_frames = 500
   endblock


Modify the line with ``max_frame = 500``
to use a different value, such as ``1000``.
Note that you could also make this change
in the project file by appending the following line:

.. code-block:: bash

   feature_tracker:max_frames = 1000

Note that the ``max_frames`` parameter is in the ``feature_tracker`` scope
and scope must be specified either using ``block``/``endblock`` notation
or with a prefix before a colon.

Printing all KLV metadata
===========================

The TeleSculptor application loads KLV metadata and display it in a viewer,
but there is no way to export this data in batch.
However, the installer does provide the kwiver command line tool
that has an applet that will print out all metadata in a video.
This applet is called :path:`dump_klv`.
The default installation path is:

.. parsed-literal::
  :class: wrap

  C: |backslash| Program Files |backslash| TeleSculptor |version|
  |backslash| bin |backslash| kwiver.exe

To run :path:`dump_klv`, open up a command prompt
(search for "cmd.exe" in the Start Menu).
Then run:

.. parsed-literal::

  "C: |backslash| Program Files |backslash| TeleSculptor |version|
  |backslash| bin |backslash| kwiver.exe" \
  dump_klv video_file.mpeg

and replace :path:`video_file.mpeg`
with the path to the video file to process.
This will print out all the metadata.
To redirect the output to a file use:

.. parsed-literal::

  "C: |backslash| Program Files |backslash| TeleSculptor |version|
  |backslash| bin |backslash| kwiver.exe" \
  dump_klv video_file.mpeg > metadata.txt
