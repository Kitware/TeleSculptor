.. image:: doc/images/TeleSculptor_Logo.png
   :width: 1024px
   :alt: TeleSculptor

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
TeleSculptor Overview
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

TeleSculptor is a cross-platform desktop application for photogrammetry.
It was designed with a focus on aerial video, such as video collected from UAVs,
and handles geospatial coordinates and can make use of metadata, if available,
from GPS and IMU sensors.  However, the software can also work with
non-geospatial data and with collections of images instead of metadata.
TeleSculptor uses Structure-from-Motion techniques to estimate camera parameters
as well as a sparse set of 3D landmarks.  It uses Multiview Stereo techniques
to estimate dense depth maps on key frame and then fuses those depth maps
into a consistent surface mesh which can be colored from the source imagery.

TeleSculptor can be installed from precompiled binaries for Linux, MacOS, and
Windows included at the bottom of the
`latest release`_ page by following the instructions in the Installation_ section.
Instructions on how to use the TeleSculptor GUI can be found in
the `User Guide <doc/TeleSculptor-v1.1-User-Guide.pdf>`_. A computer with at
least 16GB of RAM is recommended for processing most datasets.

More advanced users who wish to build the project from source should proceed to the
`Building TeleSculptor`_ section.

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Background
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

TeleSculptor provides a graphical user interface with Qt_, 3D visualization
with VTK_, and photogrammetry algorithms with KWIVER_. This project was
previously called MAP-Tk (Motion-imagery Aerial Photogrammetry Toolkit).
The MAP-Tk name is still scattered throughout the source code.
MAP-Tk started as an open source C++ collection of libraries and tools for
making measurements from aerial video.  The TeleSculptor application was added
to the project later. The original software framework and algorithm were then
refactored into KWIVER and then expanded to address broader computer vision
problems.  While KWIVER is now a more broad set of tools, TeleSculptor remains
an application focused on photogrammetry.

The advantage of the KWIVER software architecture (previously MAP-Tk) is that
it is highly modular and provides an algorithm abstraction layer
that allows seamless interchange and run-time selection of algorithms from
various other open source projects like OpenCV, VXL, Ceres Solver, and PROJ4.
The core KWIVER library (vital) and tools are light-weight with minimal
dependencies (C++ standard library, and Eigen_).  TeleSculptor is written to
depend only on the KWIVER "vital" library.  Additional capabilities are
provided by KWIVER arrows (plugin modules) that use third party libraries
to implement various abstract algorithm interfaces defined in the KWIVER vital
library.  This means that new plugins can be dropped into TeleSculptor to
enable alternative or new functionality by adjusting some settings in a
configuration file.  While TeleSculptor provides a default workflow that works
out of the box, it is not just an end user tool.  It is designed to be highly
configurable to support research into to algorithms and new problem domains.

The screenshots below show TeleSculptor
running on example videos from the `VIRAT Video Dataset`_,
`CLIF 2007 Dataset`_, and other public data sets.  More information about this
example data can be found in the `examples <examples>`_ directory.

While the initial software implementation relies on batch post-processing
of aerial video, our intent is to move to an online video stream processing
framework and optimize the algorithm to run in real-time.


.. image:: /doc/screenshot/telesculptor_screenshot_macos.png
   :alt: MacOS Screenshot
.. image:: /doc/screenshot/telesculptor_screenshot_windows.png
   :alt: Windows Screenshot
.. image:: /doc/screenshot/telesculptor_screenshot_linux.png
   :alt: Linux Screenshot


Installation
============
If you have downloaded an installer from the
`latest release`_
you can simply install TeleSculptor according to the instructions for your
operating system described below. If you are building TeleSculptor from source
you should proceed to `Building TeleSculptor`_ to create the installer before
completing the installation.

**Windows:** run the installer executable (exe) and follow the prompts in the
installer dialog. Administrative permission is required.

**Mac:** open the disk image (dmg), accept the license terms, then drag the
TeleSculptor application into the Applications folder.

**Linux:** open a bash/cmd shell and run the self extracting installer script
(sh). You can view additional installation options using
``./TeleSculptor-<version>-Linux-x86_64.sh --help``

The remainder of this document is aimed at developers who wish to build the
project from source or run command line tools.  For end users looking for
instruction on running the GUI application please read the
`User Guide <doc/TeleSculptor-v1.1-User-Guide.pdf>`_.


Building TeleSculptor
=====================

TeleSculptor requires C++11 compliant compiler
(e.g. GCC 4.8.1, Clang 3.3, Visual Studio 2015).
TeleSculptor uses CMake (www.cmake.org) for easy cross-platform compilation. The
minimum required version of CMake is 3.9.5, but newer versions are recommended.

Building
--------
The build is directed by CMake to ensure it can be built on various platforms.
The code is built by a CMake 'superbuild', meaning as part of the build,
CMake will download and build any dependent libraries needed by TeleSculptor.
The build is also out of source, meaning the code base is to be separate from
the build files.  This means you will need two folders, one for the source code
and one for the build files.
Here is the quickest way to build via a cmd/bash shell.

Before building on Linux systems you must install the following packages:

.. code-block :: bash

  sudo apt-get install build-essential libgl1-mesa-dev libxt-dev
  sudo apt-get install libexpat1-dev libgtk2.0-dev liblapack-dev

On Linux, to optionally build with Python and help menu documentation you will
also need to install the following:

.. code-block :: bash

  sudo apt-get install python3-dev python3-docutils

Set up the folder structure and obtain the source files. This can be done with
git or by downloading the files and extracting them. Then setup the folder(s)
to build the binary files.

.. code-block :: bash

  mkdir telesculptor
  cd telesculptor

  ## Place the code in a directory called src
  # Using git, clone into a new directory called src
  git clone https://github.com/Kitware/TeleSculptor.git src
  # Or unzip into a new directory called src
  unzip <file name>.zip src

  ## Create the folder where we will build the binaries
  mkdir builds
  cd builds
  # Instead of just one builds folder you can to make subfolders here for
  # different builds, for example: builds/debug and builds/release.
  # Each folder would then be built following the steps below but with different
  # configuration options

Generate the makefile/msvc solution to perform the superbuild using cmake.
A description of the configuration options can be found in `CMake Options`_.

.. code-block :: bash

  # From the build directory provide cmake the path to the source directory,
  # which can be relative or absolute.
  # Specify configurable options by prefacing them with the -D flag
  cmake -DCMAKE_BUILD_TYPE:STRING=Release ../src
  # Alternatively, you can use the 'ccmake' command line tool allows for
  # interactively selecting CMake options. This can be installed with
  # 'sudo apt-get install cmake-curses-gui'
  ccmake ../src
  # As a final option, you can use the the CMake GUI you can set the source and
  # build directories accordingly and then press the "Configure" and “Generate”
  # buttons

Build the installer target/project

.. code-block :: bash

  # On Linux/OSX/MinGW
  make
  # Once the Superbuild is complete, the telesculptor makefile will be placed in
  # the build/external/telesculptor-build directory

  # For MSVC
  # Open the TeleSculptor-Superbuild.sln, choose your build configuration,
  # from the 'Build' menu choose 'Build Solution'
  # When the build is complete you may close this solution.
  # To edit TeleSculptor code, open the
  # build/external/telesculptor-build/TeleSculptor.sln

You have now built a TeleSculptor installer similar to what is provided in the
`latest release`_ section. To install TeleSculptor on you machine, follow the
instructions above in `Installation`_.

CMake Options
-------------

================================== ===================================================
``CMAKE_BUILD_TYPE``               The compiler mode, usually ``Debug`` or ``Release``
``TELESCULPTOR_ENABLE_CUDA``       Enable GPU acceleration with CUDA
``TELESCULPTOR_ENABLE_PYTHON``     Enable Python bindings in KWIVER
``TELESCULPTOR_ENABLE_MANUALS``    Turn on building the user documentation
``TELESCULPTOR_ENABLE_TOOLS``      Build the command line tools
``TELESCULPTOR_ENABLE_TESTING``    Build the unit tests
``TELESCULPTOR_SUPERBUILD``        Build as a superbuild (build Fletch and KWIVER)
================================== ===================================================

Mulit-Configuration Build Tools
'''''''''''''''''''''''''''''''

By default the CMAKE_BUILD_TYPE is set to Release.

Separate directories are required for Debug and Release builds, requiring CMake
to be run for each.

Even if you are using a Multi-Configuration build tool (like MSVC) to build
Debug you must select the Debug CMAKE_BUILD_TYPE. (On Windows in order to debug
a project all dependent projects must be build with Debug information.)

For MSVC users wanting a RelWithDebInfo build we recommend you still choose
Release for the superbuild.  Release and RelWithDebInfo are compatible with each
other, and Fletch will build its base libraries as Release.  MSVC solutions will
provide both Release and RelWithDebInfo configuration options. You will need to
open the ``<build/directory>/external/kwiver-build/KWIVER.sln`` and build this
solution with the RelWithDebInfo configuration.


TeleSculptor
''''''''''''

The TeleSculptor GUI application is enabled by default,
and all dependencies will be built by the Superbuild.

Documentation
'''''''''''''

If ``TELESCULPTOR_ENABLE_MANUALS`` is enabled, and CMake finds all dependencies,
then the user manuals are built as part of the normal build process under the target
"manuals".  The GUI manual can be viewed from inside the GUI by choosing the
"TeleSculptor User Manual" action from the "Help" menu.

To build the user manual(s), you need:

* Python
    version 3.4 or greater
    http://www.python.org/

* Docutils
    version 0.11 or greater
    http://docutils.sourceforge.net/

(At present, only the GUI has a user manual.  Other manuals may be added in the
future.)

Testing
'''''''

Continuous integration testing is provided by CDash_.
Our `MAP-Tk dashboard <https://open.cdash.org/index.php?project=MAPTK>`_
hosts nightly build and test results across multiple platforms including
Windows, Mac, and Linux.

Anyone can contribute a build to this dashboard using the
`dashboard script <CMake/dashboard-scripts/TeleSculptor_common.cmake>`_
provided.  Follow the instructions in the comments.

`Travis CI`_ is also used for continued integration testing.
Travis CI is limited to a single platform (Ubuntu Linux), but provides
automated testing of all topic branches and pull requests whenever they are
created.

============================= =============
Travis CI **master** branch:  |CI:master|_
Travis CI **release** branch: |CI:release|_
============================= =============

Advanced Build
--------------

TeleSculptor is built on top of the KWIVER_ toolkit, which is in turn built on
the Fletch_ super build system.  As mentioned above, to make it easier to build
TeleSculptor, a "super-build" is provided to build both KWIVER and Fletch.
But, if you wish, you may point the TeleSculptor build to use your own KWIVER
builds.

If you would like TeleSculptor to use a prebuilt version of KWIVER, specify the
kwiver_DIR flag to CMake.  The kwiver_DIR is the KWIVER build directory root,
which contains the kwiver-config.cmake file.

.. code-block :: bash

    $ cmake ../../src -DCMAKE_BUILD_TYPE=Release -Dkwiver_DIR:PATH=<path/to/kwiver/build/dir>

You must ensure that the specified build of KWIVER was built with at least the following options set:

The required KWIVER flags can be found in this file : `<CMake/telesculptor-external-kwiver.cmake>`_

The required Fletch flags can be found in this file : `<CMake/telesculptor-external-fletch.cmake>`_

Overview of Directories
=======================

======================= ========================================================
``CMake``               contains CMake helper scripts
``config``              contains reusable default algorithm configuration files
``doc``                 contains release notes, manuals, and other documentation
``examples``            contains pointers to example public datasets to use
``gui``                 contains the visualization GUI source code and headers
``gui/icons``           contains the visualization GUI icon resources
``maptk``               contains the maptk library source and headers
``packaging``           contains support files for CPack packaging
``scripts``             contains Python helper scripts
``plugins/blender``     contains Python plug-ins for Blender
``plugins/sketchup``    contains Ruby plug-ins for SketchUp
``tests``               contains testing framework and tests for each module
``tools``               contains source for command line utilities
======================= ========================================================

MAP-Tk Tools
============

MAP-Tk command line tools are placed in the ``bin`` directory of the build
or install path.  These tools are described below.  Note that these tools are
in the process of being migrated to KWIVER and will leave this repository soon.
Continued support is not guaranteed and behavior may diverge from documentation.


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

TeleSculptor is a component of Kitware_'s collection of open source computer
vision tools and part of the KWIVER_ ecosystem. Please join the
`kwiver-users <http://public.kitware.com/mailman/listinfo/kwiver-users>`_
mailing list to discuss or to ask for help with using TeleSculptor.
For less frequent announcements about TeleSculptor and other KWIVER components,
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
.. _Qt: https://www.qt.io/
.. _Travis CI: https://travis-ci.com/
.. _VisualSFM: http://ccwu.me/vsfm/
.. _VTK: https://vtk.org/
.. _latest release: https://github.com/Kitware/TeleSculptor/releases/latest

.. Appendix II: Text Substitutions
.. ===============================

.. |>=| unicode:: U+02265 .. greater or equal sign

.. |CI:master| image:: https://travis-ci.com/Kitware/TeleSculptor.svg?branch=master
.. |CI:release| image:: https://travis-ci.com/Kitware/TeleSculptor.svg?branch=release

.. _CI:master: https://travis-ci.com/Kitware/TeleSculptor
.. _CI:release: https://travis-ci.com/Kitware/TeleSculptor
