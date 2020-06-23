############################################
             TeleSculptor Examples
############################################

We provide two suggested video clips and a description of how to run a
project in the TeleSculptor GUI. For a more thorough description of
TeleSculptor workflows, please see the
`User Guide <../doc/TeleSculptor-v1.1-User-Guide.pdf>`_.

Data
====
We recommend that new users try one of the following videos released as part
of the `VIRAT Video Dataset`_.
You will need to accept the Terms of Use before you proceed to the download.


09172008flight1tape3_2.mpg_: The aircraft makes about one complete orbit over
a site in Fort A.P. Hill, Virginia. The stare point remains fixed on the center
of the scene and the field of view is quite narrow. The scene is composed
primarily of buildings and vehicles.

09152008flight2tape2_4.mpg_: This is video of the same region but taken in a
different style. The field of view is much larger and the stare point moves
substantially. In addition to the buildings and vehicles which are the focus of
the other clip, this video includes wide shots of roads and vegetation.

If you are considering running TeleSculptor on different data, there are several things to
keep in mind. The first is that TeleSculptor is optimized for aerial data and
results may vary on data collected in other ways.
Additionally, the way to data is captured has a significant impact on the
quality of the model. It is important to have multiple views of the same regions
from different directions and if possible the camera should complete a full rotation about the scene.
It is best not to change the zoom of your lens and if you do it should be done
slowly and gradually. Finally, you should avoid switching between different
modalities, for example recording part of your video in color and the rest in infrared.
If you are capturing images rather than video, it is important that there is
substantial overlap in the content seen between different images.

Setup
=====
It is recommended that you create a seperate project folder for each video and
place the video in the folder. Now open the TeleSculptor GUI and click File -> "New Project"
and select your folder. This will create a ``<Project Name>.conf``
file in that directory which can be opened to restore the project state in the
future. Now load the data by clicking File -> Import -> Imagery. After the
video is loaded, you should see a thumbnail view in the upper right and you can
scrub through the video using the bottom bar.

Compute
=======
The Compute menu in the upper left allows you to perform a series of different
processing steps. If you would like to run all the steps to produce a full 3D
model from you data, simply click Compute -> "Run End-to-End".

View
====
At any point you can scrub through the video using the bottom bar and you will
see overlays of additional information which has been computed, in addition to
the 3D model if generated. View -> "Play Slideshow" progresses through the
video. View -> Match shows you a visualization of the pairwise strength of
association between different frames.

.. Appendix I: References
.. ======================

.. _VIRAT Video Dataset: http://www.viratdata.org/
.. _Kitware: http://www.kitware.com/
.. _09152008flight2tape2_4.mpg: https://data.kitware.com/#item/56f580488d777f753209c72f
.. _09172008flight1tape3_2.mpg: https://data.kitware.com/#item/5ef11b419014a6d84ed53971
