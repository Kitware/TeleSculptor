############################################
             MAP-Tk Examples
############################################

This directory contains example configuration files to demonstrate how to run
MAP-Tk tools on publically available data sets.  Each subdirectory corresponds
to a data set and contains configuration files that have been tested on that
data set.  The configuration file names match the names of command line tools
which can run them.  The example data sets can be found on the KWIVER_ website
at http://www.kwiver.org/testData/.  These selected data sets are subsets
of larger datasets as described below.

=================== ===========================================================
kwiver_fmv_set_1_   A five-minute clip from the `VIRAT Video Dataset`_ with
                    frames sampled at 10Hz. There are 3089 frames of video at
                    720x480 resolution.  The aircraft makes about one complete
                    orbit over a site in Fort A.P. Hill, Virginia.  The center
                    of the site stays in the field of view most of the time.

kwiver_wami_set_1_  A sample from one camera of the `CLIF 2007`_ data set.  It
                    contains 495 frames from `Camera 0` (of a six-camera array)
                    at 668x1004 resolution.  This sample has been down-sampled
                    by a factor of four from the resolution of the original
                    data.  The video is about 1.5Hz and make more than two
                    orbits over Ohio State University.  Because only one camera
                    is used, the stare point is outside of the field of view.
                    Since the same locations are visited two or more times,
                    this data set is useful for evaluating loop closure.
=================== ===========================================================

The easiest way to run these examples is to download and extract the data set
files into the example configuration directory with the correspond name.  Then
run ``maptk_[tool] -c maptk_[tool].conf`` from within that directory where
``[tool]`` is replaced with one of the tool names, like ``track_features``.
Run ``maptk_track_features`` first, followed by ``maptk_bundle_adjust_tracks``.
The outputs are all written to the ``results`` subdirectory, for which an empty
directory is provided as a placeholder.

.. Appendix I: References
.. ======================

.. _CLIF 2007: https://www.sdms.afrl.af.mil/index.php?collection=clif2007
.. _KWIVER: http://www.kwiver.org/
.. _kwiver_fmv_set_1: http://www.kwiver.org/files/kwiver_fmv_set_1.tgz
.. _kwiver_wami_set_1: http://www.kwiver.org/files/kwiver_wami_set_1.tgz
.. _VIRAT Video Dataset: http://www.viratdata.org/
