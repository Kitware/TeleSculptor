############################################
                   MAP-Tk
############################################
.. image:: /gui/icons/64x64/telesculptor.png
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Motion-imagery Aerial Photogrammetry Toolkit
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

MAP-Tk started as an open source C++ collection of libraries and tools for
making measurements from aerial video.  Initial capability focused on
estimating the camera flight trajectory and a sparse 3D point cloud of a scene.
These products are jointly optimized via sparse bundle adjustment and are
geo-localized if given additional control points or GPS metadata.

This project has similar goals as projects like Bundler_ and VisualSFM_.
However, the focus here in on efficiently processing aerial video rather than
community photo collections. Special attention has been given to the case where
the variation in depth of the 3D scene is small compared to distance to the
camera.  In these cases, planar homographies can be used to assist feature
tracking, stabilize the video, and aid in solving loop closure problems.

MAP-Tk uses the KWIVER_ software architecture.  Originally developed for
MAP-Tk, KWIVER is highly modular and provides an algorithm abstraction layer
that allows seamless interchange and run-time selection of algorithms from
various other open source projects like OpenCV, VXL, Ceres Solver, and PROJ4.
The core library and tools are light-weight with minimal dependencies
(C++ standard library, KWIVER_ vital, and Eigen_).  The tools are written to depend
only on the MAP-Tk and KWIVER vital libraries.  Additional capabilities are
provided by KWIVER arrows (plugin modules) that use third party libraries
to implement various abstract algorithm interfaces defined in the KWIVER vital
library.  Earlier versions of MAP-Tk contained these core data structures,
algorithms, and plugins, but these have since been moved to KWIVER for easier
reuse across projects.  What remains in this repository are the tools, scripts,
and applications required to apply KWIVER algorithms to photogrammetry problems.
As MAP-Tk capabilities have continued to migrate up into KWIVER this repository
has become less of a "toolkit" and more of an end user application that uses
the KWIVER toolkit.  Additionally the capabilities are starting to branch out
beyond aerial data.  As a result, we are transitioning away from the MAP-Tk
name as this repository becomes more about the GUI application named
TeleSculptor.

TeleSculptor is a GUI application built on Qt.  It provides a graphical
interface to run photogrammetry algorithms and assist with visualization of
data and results with the help of VTK.  The screenshots below show TeleSculptor
running on example videos from the `VIRAT Video Dataset`_,
`CLIF 2007 Dataset`_, and other public data sets.  More information about this
example data can be found in the `examples <examples>`_ directory.

.. image:: /doc/screenshot/telesculptor_screenshot_macos.png
   :alt: MacOS Screenshot
.. image:: /doc/screenshot/mapgui_screenshot_windows.png
   :alt: Windows Screenshot
.. image:: /doc/screenshot/telesculptor_screenshot_linux.png
   :alt: Linux Screenshot

TeleSculptor now supports visualization of depth maps, but compution of
depth maps is not yet supported by KWIVER.  Instead, the cameras computed
by MAP-Tk can be used with a fork of PlaneSweepLib_ that reads in the cameras
and images and produces depthmaps that the GUI can load.  We are working on
extending MAP-Tk TeleSculptor to compute depth maps directly.

While the initial software implementation relies on batch post-processing
of aerial video, our intent is to move to an online video stream processing
framework and optimize the algorithm to run in real-time.


Overview of Directories
=======================

======================= ========================================================
``CMake``               contains CMake helper scripts
``config``              contains reusable default algorithm configuration files
``doc``                 contains release notes, manuals, and other documentation
``examples``            contains example tool configuration for public datasets
``gui``                 contains the visualization GUI source code and headers
``gui/icons``           contains the visualization GUI icon resources
``maptk``               contains the maptk library source and headers
``packaging``           contains support files for CPack packaging
``scripts``             contains Python helper scripts
``scripts/blender``     contains Python plug-ins for Blender
``tests``               contains testing framework and tests for each module
``tools``               contains source for command line utilities
======================= ========================================================


Building MAP-Tk
===============

MAP-Tk uses CMake (www.cmake.org) for easy cross-platform compilation. The
minimum required version of CMake is 3.0, but newer versions are recommended.

As with KWIVER, MAP-Tk requires C++11 compliant compiler
(e.g. GCC 4.8, Visual Studio 2015).


Running CMake
-------------

We recommend building MAP-Tk out of the source directory to prevent mixing
source files with compiled products.  Create a build directory in parallel
with the MAP-Tk source directory.  From the command line, enter the
empty build directory and run::

    $ ccmake /path/to/maptk/source

where the path above is the location of your MAP-Tk source tree.  The ccmake
tool allows for interactive selection of CMake options.  Alternatively, using
the CMake GUI you can set the source and build directories accordingly and
press the "Configure" button.


CMake Options
-------------

============================== =================================================
``CMAKE_BUILD_TYPE``           The compiler mode, usually ``Debug`` or ``Release``
``CMAKE_INSTALL_PREFIX``       The path to where you want MAP-Tk to install

``MAPTK_ENABLE_MANUALS``       Turn on building the user documentation (manuals)
``MAPTK_ENABLE_DOCS``          Turn on building the Doxygen documentation
``MAPTK_INSTALL_DOCS``         Install Doxygen documentation (requires above
                               enabled)
``MAPTK_ENABLE_TESTING``       Build the unit tests

``kwiver_DIR``                 Path to the KWIVER build or install tree
``qtExtensions_DIR``           Path to the QtExtension build or install tree
============================== =================================================


Dependencies
------------

MAP-Tk has minimal required dependencies at the core level.  Enabling
plugins adds additional capabilities as well as additional dependencies.
Some functionality is duplicated between modules to provide choices.
Feature tracking requires OpenCV or VisCL. Bundle adjustment requires
Ceres Solver or VXL.  Geographic transformations require PROJ4.

Required
''''''''

The only hard dependencies of MAP-Tk are on the C++ standard library,
KWIVER_ (|>=| 1.1), and Eigen_ (|>=| 3.0; also required by KWIVER).

Recommended KWIVER Plugins
''''''''''''''''''''''''''

When building KWIVER for use in MAP-Tk there are several arrows (plugins) that
should be enabled to provide maximum capability to MAP-Tk.  The KWIVER arrows
are not a build-time dependency of MAP-Tk, but are required at run-time to
provide algorithm implementations to run.  The following KWIVER arrows provide
algorithms which are optionally used by MAP-Tk:

* Core
    algorithm implementations with no additional dependencies

* Ceres
    supplies bundle adjustment using Ceres Solver
    http://ceres-solver.org/

* OpenCV
    supplies feature detectors, descriptors, matcher; homography and
    fundamental matrix estimators; image I/O, and more.
    http://opencv.org/

* PROJ
    provides geographic transforms (e.g. Lat/Lon to UTM)
    http://trac.osgeo.org/proj/:

* VisCL
    experimental code for OpenCL acceleration
    (currently not recommend for most users)
    https://github.com/Kitware/VisCL

* VXL
    supplies a simple bundle adjuster, image I/O, homgraphy and
    fundamental matrix estimation, and more.
    (note: requires unreleased version, use Fletch_ to build)
    http://vxl.sourceforge.net/

TeleSculptor
''''''''''''

The MAP-Tk TeleSculptor GUI application is an optional (but recommended)
part of the MAP-Tk build.  It has additional dependencies.
To build the TeleSculptor, you need:

* Qt 4
    version 4.8.0 or greater (4.8.6 or greater recommended, provided by Fletch)
    http://www.qt.io/

* qtExtensions
    no versioned releases as of writing; use master branch
    http://www.github.com/kitware/qtextensions

* VTK
    version 6.2 (provided by FLetch)
    http://www.vtk.org/

Most of the dependencies for KWIVER and MAP-Tk can be provided by a
meta-project called Fletch_.  Fletch uses CMake to fetch, configure,
and build various third party packages such that they work together
in a consistent way across platforms.  We recommend that you use Fletch
to build Ceres, Eigen, OpenCV, PROJ, Qt, VTK, and VXL and their dependencies.
Next build KWIVER and set "fletch_DIR" in CMake to point to your Fletch build.
Enable the arrows recommended above in the KWIVER build. Finally, build MAP-Tk
and set "kwiver_DIR" in CMake to point to your KWIVER build.

Documentation
'''''''''''''

Documentation generation is another optional component that brings in
additional dependencies.  To build the API documentation, you need:

* Doxygen
    version 1.7 or greater
    http://www.stack.nl/~dimitri/doxygen/

To build the user manual(s), you need:

* Python
    version 2.6 or greater
    http://www.python.org/

* Docutils
    version 0.11 or greater
    http://docutils.sourceforge.net/

(At present, only the GUI has a user manual.  Other manuals may be added in the
future.)

Nightly builds of the Doxygen documentation for the primary branches are here:

================================= ================================================
Nightly **master** Documentation  http://www.kwiver.org/maptk/docs/nightly/master
Nightly **release** Documentation http://www.kwiver.org/maptk/docs/nightly/release
================================= ================================================

Doxygen documentation for released versions are here:

================================= ===============================================
**MAP-Tk v0.6.1** Documentation   http://www.kwiver.org/maptk/docs/release/v0.6.1
**MAP-Tk v0.7.2** Documentation   http://www.kwiver.org/maptk/docs/release/v0.7.2
**MAP-Tk v0.8.1** Documentation   http://www.kwiver.org/maptk/docs/release/v0.8.1
**MAP-Tk v0.9.0** Documentation   http://www.kwiver.org/maptk/docs/release/v0.9.0
================================= ===============================================

Building Documentation
----------------------

If ``MAPTK_ENABLE_DOCS`` is enabled, and CMake finds, or is provided with, a
path to the Doxygen tool, then the HTML documentation is built as part of the
normal build process under the target "doxygen".  Open
``${MAPTK_BUILD_DIR}/docs/index.html`` in your browser to view the
documentation.

If ``MAPTK_ENABLE_MANUALS`` is enabled, and CMake finds, or is provided with, a
path to the Python executable which is able to import docutils, then the user
manuals are built as part of the normal build process under the target
"manuals".  The GUI manual can be viewed from inside the GUI by choosing the
"MAP-Tk TeleSculptor User Manual" action from the "Help" menu.


Testing
========
Continuous integration testing is provided by CDash_.
Our `MAP-Tk dashboard <https://open.cdash.org/index.php?project=MAPTK>`_
hosts nightly build and test results across multiple platforms including
Windows, Mac, and Linux.

Anyone can contribute a build to this dashboard using the
`dashboard script <CMake/dashboard-scripts/MAPTK_common.cmake>`_
provided.  Follow the instructions in the comments.


`Travis CI`_ is also used for continued integration testing.
Travis CI is limited to a single platform (Ubuntu Linux), but provides
automated testing of all topic branches and pull requests whenever they are created.

============================= =============
Travis CI **master** branch:  |CI:master|_
Travis CI **release** branch: |CI:release|_
============================= =============

MAP-Tk Tools
============

MAP-Tk command line tools are placed in the ``bin`` directory of the build
or install path.  These tools are described below.


Summary of MAP-Tk Tools
-----------------------

The primary tools are ``maptk_track_features`` and
``maptk_bundle_adjust_tracks``. Together these form the sparse bundle
adjustment pipeline.  The other tools are for debugging and analysis purposes.

``maptk_detect_and_describe``
  This optional tool pre-computes feature points and descriptors on each frame
  of video and caches them on disk.  The same is also done in the
  ``maptk_track_features``, so this step is not required.  However, this tool
  makes better use of threading to process all frames in parallel.

``maptk_track_featues``
  Takes a list of images and produces a feature tracks file.

``maptk_bundle_adjust_tracks``
  Takes feature tracks and produces cameras (KRTD files) and 3D points (PLY
  file). Can also take input POS files or geo-reference points and produce
  optimized POS files.

``maptk_apply_gcp``
  This tool takes an existing solution from ``maptk_bundle_adjust_tracks``
  and uses provided ground control points (GCPs) to fit a 3D similarity
  transformation to align the solution to the GCPs.  The same is done in
  the bundle adjust tool, but this tool lets you update and reapply GCPs
  without recomputing bundle adjustment.

``maptk_pos2krtd``
  Takes POS files and directly produces KRTD.

``maptk_analyze_tracks``
  Takes images and feature tracks and produces tracking statistics or images
  with tracks overlaid.

``maptk_estimate_homography``
  Estimates a homography transformation between two images, outputting a file
  containing the matrices.


Running MAP-Tk Tools
--------------------

Each MAP-Tk tool has the same interface and accepts three command line
arguments:

* ``-c`` to specify an input configuration file
* ``-o`` to output the current configuration to a file
* ``-h`` for help (lists these options)

Each tool has all of its options, including paths to input and output files,
specified in the configuration file.  To get started, run one of the tools
like this::

    $ maptk_track_features -o config_file.conf

This will produce an initial set of configuration options.  You can then edit
``config_file.conf`` to specify input/output files, choices of algorithms, and
algorithm parameters.  Just as in CMake, configuring some parameters will
enable new sub-parameters and you need to re-run the tool to get the updated
list of parameters.  For example::

    $ maptk_track_features -c config_file.conf -o config_file.conf

The above command will overwrite the existing config file with a new file.
Ordering of entries and comments are not preserved.  Use a different output
file name to prevent overwriting the original.  Continue to adjust parameters
and re-run the above command until the tool no longer reports the message::

    ERROR: Configuration not valid.

Note that the config file itself contains detail comments documenting each
parameter.  For each abstract algorithm you must specify the name of variant
to use, but the list of valid names (based on which modules are compiled)
is provided directly in the comment for easy reference. When the config file
is complete and valid, run the tool one final time as::

    $ maptk_track_features -c config_file.conf

An easier way to get started is to use the sample configuration files for each
tool that are provided in the ``examples`` directory.  These examples use
recommended default settings that are known to produce useful results on some
selected public data samples.  The example configuration files include the
default configuration files for each algorithm in the ``config`` directory.


Getting Help
============

MAP-Tk is a component of Kitware_'s collection of open source computer vision
tools known as KWIVER_. Please join the
`kwiver-users <http://public.kitware.com/mailman/listinfo/kwiver-users>`_
mailing list to discuss MAP-Tk or to ask for help with using MAP-Tk.
For less frequent announcements about MAP-Tk and other KWIVER components,
please join the
`kwiver-announce <http://public.kitware.com/mailman/listinfo/kwiver-announce>`_
mailing list.


Acknowledgements
================

The authors would like to thank AFRL/Sensors Directorate for their support
of this work via SBIR Contract FA8650-14-C-1820. This document is approved for
public release via 88ABW-2015-2555.


.. Appendix I: References
.. ======================

.. _VIRAT Video Dataset: http://www.viratdata.org/
.. _CLIF 2007 Dataset: https://www.sdms.afrl.af.mil/index.php?collection=clif2007
.. _Bundler: http://www.cs.cornell.edu/~snavely/bundler/
.. _CDash: http://www.cdash.org/
.. _Eigen: http://eigen.tuxfamily.org/
.. _Fletch: https://github.com/Kitware/fletch
.. _Kitware: http://www.kitware.com/
.. _KWIVER: http://www.kwiver.org/
.. _PlaneSweepLib: https://github.com/bastienjacquet/PlaneSweepLib
.. _Travis CI: https://travis-ci.org/
.. _VisualSFM: http://ccwu.me/vsfm/

.. Appendix II: Text Substitutions
.. ===============================

.. |>=| unicode:: U+02265 .. greater or equal sign

.. |CI:master| image:: https://travis-ci.org/Kitware/maptk.svg?branch=master
.. |CI:release| image:: https://travis-ci.org/Kitware/maptk.svg?branch=release

.. _CI:master: https://travis-ci.org/Kitware/maptk
.. _CI:release: https://travis-ci.org/Kitware/maptk
