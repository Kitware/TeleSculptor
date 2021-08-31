.. _glossary:

===============================================================================
Glosary
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

Camera:
  In TeleSculptor, a camera
  refers primarily to the model
  which describes the intrinsic and extrinsic parameters of a camera
  at a given frame.
  Intrinsic parameters include focal length and lens distortion.
  Extrinsic parameters are the camera pose |--| orientation and position.
  A frame may have at most one camera pose,
  which is unique to that frame.
  While extrinsic parameters are never shared across frames,
  intrinsic parameters may or may not be share depending on the algorithm
  that produced them.

  In the GUI, cameras are represented as rectangular pyramids.
  The peak of the pyramid is the camera center of projection.
  The rectangle base is proportional to the image aspect ratio,
  and a triangle attached to the base indicates
  which direction is "up" in the image.
  The pyramid visualizes the field of view of the camera.


Feature:
  A feature is a location that corresponds to a salient point in an image.
  Features have enough visual texture that they can be reliably localized
  in images across time.
  Features are also known as interest points or corner points.

Track:
  A track, or feature track, is a collection of correlated features over time;
  that is, detected feature points
  estimated to correspond to the same landmark.

Landmark:
  A landmark is an estimated 3D world point that,
  when projected into the images,
  gave rise to an observed feature track.

Residual:
  A residual, in general, is the difference between an observed value and an
  estimated value\ [#er]_. In TeleSculptor, the observed value is typically a
  detected feature point, and the estimated value is the projection of its
  corresponding landmark into the image.

Ground Control Point (GCP):
  A ground control point, or GCP, is a 3D point that is
  manually placed in the scene by a user.
  GCPs are often attached to an identifiable location
  from a reference image or map.
  The user can assign the true geodetic location
  (latitude, longitude, elevation) to a GCP,
  and that GCP can then serve as a constraint when geo-registering the model.

Camera Registration Point (CRP):
  A camera registration point, or CRP, is a 2D point that is
  manually placed in one or more images by a user.
  CRPs are associated with a GCP point and can tie that GCP
  to multiple images, even when no camera model has been estimated.
  While a GCP is the manually placed version of a landmark,
  a CRP is the manually placed version of a feature track.
  A collection of CRPs can be used to manually estimate a camera model
  when automated feature matching is not possible.

----

.. [#er] https://en.wikipedia.org/wiki/Errors_and_residuals_in_statistics
