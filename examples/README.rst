############################################
             TeleSculptor Examples
############################################

This directory contains a walkthrough of running the TeleSculptor GUI
application on two publically video clips. For a more thorough description of
TeleSculptor workflows, please see the
`User Guide <../doc/TeleSculptor-v1.1-User-Guide.pdf>`_.

Data
====
We recommend that new users try one of the following videos released as part
of the `VIRAT Video Dataset`_.
Before accessing the data for the first time you will need to accept
the Terms of Use, and then you will be able to download the videos.


09172008flight1tape3_2.mpg_: The aircraft makes about one complete orbit over
a site in Fort A.P. Hill, Virginia. The center of the site stays in the field of
view most of the time. Compared to the other video, the field of view is
significantly narrower.

09152008flight2tape2_4.mpg_: Is a wide field of view video of A.P. Hill. It
has some panning from side to side but maintains a fairly consistent stare point
on the main collection of buildings.

If you are considering running TeleSculptor on different data, there are several things to
keep in mind. The first is that TeleSculptor is optimized for reconstructing 3D
models from aerial images or video and results may vary on other types of data.
Additionally, the way to data is captured has a significant impact on the
quality of the model. It is important to have multiple views of the same regions
from different directions, relatively consistent
zoom, and only capture using one channel rather than switching between different
ones such as color to infrared. If capturing images, you should make sure there
is substantial overlap between frames.

Setup
=====
It is recommended that you create a project folder for each video and place the
data there. Now open the TeleSculptor GUI and click File -> "New Project" or
and select your folder. This will create a ``<Project Name>.conf``
file in that directory which can be opened to restore the project state in the
future. Now load the data by clicking File -> Import -> Imagery. After the
video is loaded, you should see a thumbnail view in the upper right and you can
scrub through using the bottom bar.

Compute
=======
The Compute menu in the upper left allows you to perform a series of different
processing steps. If you would like to run all the steps to produce a full 3D
model from you data, simply click Compute -> "Run End-to-End".

View
====
At any point you can scrub through the video using the bottom bar and you will
see overlays of additional information which has been computed, in addition to
the 3D model if applicable. View -> "Play Slideshow" progresses through the
video. View -> Match shows you a visualization of the pairwise strength of
association between different frames.

.. Appendix I: References
.. ======================

.. _VIRAT Video Dataset: http://www.viratdata.org/
.. _Kitware: http://www.kitware.com/
.. _09152008flight2tape2_4.mpg: https://data.kitware.com/#item/56f580488d777f753209c72f
.. _09172008flight1tape3_2.mpg: https://data.kitware.com/#item/5ef11b419014a6d84ed53971
