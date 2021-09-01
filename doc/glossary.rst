.. _glossary:

===============================================================================
Glossary
===============================================================================

Frame:
  In order to work, photogrammetry requires multiple images of a scene
  that have been captured from distinct angles.
  For simplicity, TeleSculptor
  refers to these distinct images as frames,
  whether or not they are derived
  from a single, continuous video.
  Although a frame corresponds to a camera pose,
  the term "frame" is used to indicate
  instances where the pose may not yet be known.

Camera Model:
  In TeleSculptor, a camera model
  (often shortened to "camera")
  describes the intrinsic and extrinsic parameters
  of a camera at a given frame.
  Intrinsic parameters include focal length and lens distortion.
  Extrinsic parameters are the camera pose |--| orientation and position.
  A frame may have at most one camera pose,
  which is unique to that frame.
  While extrinsic parameters are never shared across frames,
  intrinsic parameters may or may not be shared
  depending on the algorithm that produced them.

  In the GUI, cameras are represented as rectangular pyramids.
  The peak of the pyramid is the camera center of projection.
  The rectangle base is proportional to the image aspect ratio,
  and a triangle attached to the base
  indicates which direction is "up" in the image.
  The pyramid visualizes the field of view of the camera.

Feature:
  A feature describes a salient point in an image.
  Features have enough visual texture
  that they can be reliably localized
  in images across time.
  Features are also known
  as interest points
  or corner points.

Track:
  A (feature) track is a collection of correlated features;
  that is, detected feature points
  estimated to correspond to the same landmark.

Landmark:
  A landmark is an estimated 3D world point that,
  when projected into the images,
  gave rise to an observed feature track.

Residual:
  A residual, in general, is the difference
  between an observed value and an estimated value\ [#er]_.
  In TeleSculptor, the observed value
  is typically a detected feature point,
  and the estimated value is the projection
  of its corresponding landmark into the image.

Ground Control Point (GCP):
  A |gcp| is a 3D point
  that is manually placed in the scene by a user.
  Ground control points are often attached
  to an identifiable location from a reference image or map.
  The user can assign the true geodetic location
  (latitude, longitude, elevation) to a |gcp|,
  and that |gcp| can then serve as a constraint
  when geo-registering the model.

Camera Registration Point (CRP):
  A |crp| is a 2D point
  that is manually placed by the user
  in one or more frame images.
  Camera registration points are associated with a |gcp|
  and can tie that |gcp| to multiple images,
  even when no camera model has been estimated.
  Whereas a |gcp| is the manually placed version of a landmark,
  a |crp| is the manually placed version of a feature track.
  A collection of |crp|\ s can be used
  to manually estimate a camera model
  when automated feature matching is not possible.

----

.. [#er] https://en.wikipedia.org/wiki/Errors_and_residuals_in_statistics
