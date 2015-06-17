#                MAP-Tk                    #
Motion-imagery Aerial Photogrammetry Toolkit

--------------------------------------------

MAP-Tk is an open source C++ collection of libraries and tools for making
measurements from aerial video.  Initial capability focuses on estimating
the camera flight trajectory and a sparse 3D point cloud of the scene.
These products are jointly optimized via sparse bundle adjustment and are
geo-localized if given additional control points or GPS metadata.

This project has similar goals as projects like
[Bundler](http://www.cs.cornell.edu/~snavely/bundler/) and
[VisualSFM](http://ccwu.me/vsfm/).  However, the focus here in on efficiently
processing aerial video rather than community photo collections.
Special attention has been given to the case where the variation in depth of
the 3D scene is small compared to distance to the camera.  In these cases,
planar homographies can be used to assist feature tracking, stabilize the
video, and aid in solving loop closure problems.

The MAP-Tk software architecture is highly modular and provides an algorithm
abstraction layer that allows seamless interchange and run-time selection of
algorithms from various other open source projects like OpenCV, VXL,  VisCL,
and PROJ4.  The core library is light-weight with minimal dependencies
(C++ standard library plus some Boost and Eigen components).  The tools are
written to depend only on the MAP-Tk core library.  Additional capabilities are
provided in plugin modules that use 3rd party libraries to implement various
abstract algorithm interfaces defined in the core.  Plugin modules may also
implement additional versions of core MAP-Tk data structures.

While the initial software implementation relies on batch post-processing
of aerial video.  Our intent is to move to an online video stream processing
framework and optimize the algorithms to real-time performance for use
onboard unmanned aerial vehicles.


Overview of Directories
=======================

* CMake               -- contains CMake helper scripts
* doc                 -- contains release notes and other documentation
* maptk               -- contains the core library source and headers
* maptk/algo          -- contains abstract algorithm definitions
* maptk/plugins/core  -- contains core plugin source and headers
* maptk/plugins/ocv   -- contains the OpenCV plugin source and headers
* maptk/plugins/proj  -- contains the PROJ4 plugin source and headers
* maptk/plugins/viscl -- contains the VisCL plugin source and headers
* maptk/plugins/vxl   -- contains the VXL plugin source and headers
* scripts             -- contains Python helper scripts
* scripts/blender     -- contains Python plug-ins for Blender
* tests               -- contains testing framework and test for each module
* tools               -- contains source for command line utilities
* tools/config        -- contains example config files for command line utilities



Building MAP-Tk
===============

MAP-Tk uses CMake (www.cmake.org) for easy cross-platform compilation. The
minimum required version of CMake is 2.8.11, but newer versions are recommended.


Running CMake
-------------

We recommend building MAP-Tk out of the source directory to prevent mixing
source files with compiled products.  Create a build directory in parallel
with the MAP-Tk source directory.  From the command line, enter the
empty build directory and run

    $ ccmake /path/to/maptk/source

where the path above is the location of your MAP-Tk source tree.  The ccmake
tool allows for interactive selection of CMake options.  Alternatively, using
the CMake GUI you can set the source and build directories accordingly and
press the "Configure" button.


CMake Options
-------------

* BUILD_SHARED_LIBS    -- Set this on to build shared libraries (.so/.dylib/.dll)
                          Note, build shared is currently required for Windows
* CMAKE_BUILD_TYPE     -- The compiler mode, usually Debug or Release
* CMAKE_INSTALL_PREFIX -- The path to where you want MAP-Tk to install

* MAPTK_ENABLE_DOCS    -- Turn on building the Doxygen documentation
* MAPTK_INSTALL_DOCS   -- Install Doxygen documentation (requires above enabled)
* MAPTK_ENABLE_TESTING -- Build the unit tests

* MAPTK_ENABLE_OPENCV  -- Turn on building the OpenCV plugin module
* MAPTK_ENABLE_PROJ    -- Turn on building the PROJ.4 plugin module
* MAPTK_ENABLE_VISCL   -- Turn on building the VisCL plugin module
* MAPTK_ENABLE_VXL     -- Turn on building the VXL plugin module


Dependencies
------------

MAP-Tk has minimal required dependencies at the core level.  Enabling add-on
modules adds additional capabilities as well as additional dependencies.
Some functionality is duplicated between modules to provide choices.
Currently a complete set of functionalities in all tools requires either
[VisCL, VXL, and Proj] or [OpenCV, VXL, and Proj].

### Required ###

The only hard dependencies of MAP-Tk are on the C++ standard library and
[Boost](http://www.boost.org/) (>= v1.50) and
[Eigen](http://eigen.tuxfamily.org/) (>= 3.0).
Currently MAP-Tk uses the following Boost components:
system, filesystem, program_options, timer, chrono.

### Optional Plugins###

Each MAP-Tk plugin module brings in more dependencies for additional
functionality.  Dependencies for each module are:

* OpenCV - version 2.4.6 or greater
           <http://opencv.org/>
* PROJ   - version 4.7
           <http://trac.osgeo.org/proj/>
* VisCL  - experimental code (unversioned, use master branch)
           <https://github.com/Kitware/VisCL>
* VXL    - version 1.17 or greater
           <http://vxl.sourceforge.net/>
* Docs   - Doxygen version 1.7 or greater
           <http://www.stack.nl/~dimitri/doxygen/>


Building Doxygen Documentation
------------------------------

If `MAPTK_ENABLE_DOCS` is enabled, and CMake finds, or is provided with, a path
to the Doxygen tool, then the HTML documentation is built as part of the normal
build process under the target "doxygen".  Open
`${MAPTK_BUILD_DIR}/docs/index.html` in your browser to view the documentation.

Testing
=======
Continuous integration testing is provided by
[CDash](http://www.cdash.org/).
Our [MAP-Tk dashboard](https://open.cdash.org/index.php?project=MAPTK)
hosts nightly build and test results across multiple platforms including
Windows, Mac, and multiple flavors of Linux.

Anyone can contribute a build to this dashboard using the
[dashboard script](CMake/dashboard-scripts/MAPTK_common.cmake)
provided.  Follow the instructions in the comments.


[Travis CI](https://travis-ci.org/) is also used for continued integration testing.
Travis CI is limited to a single platform (currently Ubuntu 12.04), but provides
automated testing of all topic branches and pull requests whenever they are created.

* Travis CI **master** branch: [![Build Status](https://travis-ci.org/Kitware/maptk.svg?branch=master)]
(https://travis-ci.org/Kitware/maptk)

* Travis CI **release** branch: [![Build Status](https://travis-ci.org/Kitware/maptk.svg?branch=release)]
(https://travis-ci.org/Kitware/maptk)

MAP-Tk Tools
============

MAP-Tk command line tools are placed in the `bin` directory of the build
or install path.  These tools are described below.


Summary of MAP-Tk Tools
-----------------------

The primary tools are `maptk_track_features` and `maptk_bundle_adjust_tracks`.
Together these form the sparse bundle adjustment pipeline.  The other tools
are for debugging and analysis purposes.

* maptk_track_featues        -- Takes a list of images and produces a feature
                                tracks file.
* maptk_bundle_adjust_tracks -- Takes feature tracks and produces cameras
                                (KRTD files) and 3D points (PLY file).
                                Can also take input POS files or geo-reference
                                points and produce optimized POS files.
* maptk_pos2krtd             -- Takes POS files and directly produces KRTD.
* maptk_analyze_tracks       -- Takes images and feature tracks and produces
                                tracking statistics or images with tracks
                                overlaid
* maptk_estimate_homography  -- Estimates a homography transformation between
                                two images, outputting a file containing the
                                matrices.


Running MAP-Tk Tools
--------------------

Each MAP-Tk tool has the same interface and accepts three command line
arguments:

 * `-c` to specify an input configuration file
 * `-o` to output the current configuration to a file
 * `-h` for help (lists these options)

Each tool has all of its options, including paths to input and output files,
specified in the configuration file.  To get started, run one of the tools
like this:

    $ maptk_track_features -o config_file.conf

This will produce an initial set of configuration options.  You can then edit
`config_file.conf` to specify input/output files, choices of algorithms, and
algorithm parameters.  Just as in CMake, configuring some parameters will
enable new sub-parameters and you need to re-run the tool to get the updated
list of parameters.  For example

    $ maptk_track_features -c config_file.conf -o config_file.conf

The above command will overwrite the existing config file with a new file.
Ordering of entries and comments are not preserved.  Use a different output
file name to prevent overwriting the original.  Continue to adjust parameters
and re-run the above command until the tool no longer reports the message:

    ERROR: Configuration not valid.

Note that the config file itself contains detail comments documenting each
parameter.  For each abstract algorithm you must specify the name of variant
to use, but the list of valid names (based on which modules are compiled)
is provided directly in the comment for easy reference. When the config file
is complete and valid, run the tool one final time as

    $ maptk_track_features -c config_file.conf

An easier way to get started is to use the sample configuration files for each
tool that are provided in the `tools/config` directory.  These examples use
recommended default settings that are known to produce useful results.

Getting Help
============

MAP-Tk is a component of [Kitware's](http://www.kitware.com) collection of open
source computer vision tools known as [KWIVER](http://www.kwiver.org).
Please join the [kwiver-users](http://public.kitware.com/mailman/listinfo/kwiver-users)
mailing list to discuss MAP-Tk or to ask for help with using MAP-Tk.
For less frequent announcements about MAP-Tk and other KWIVER components
please join the [kwiver-announce](http://public.kitware.com/mailman/listinfo/kwiver-announce)
mailing list.

Acknowledgements
================
The authors would like to thank AFRL/Sensors Directorate for their support
of this work via SBIR Contract FA8650-14-C-1820.
This document is approved for public release via 88ABW-2015-2555.
