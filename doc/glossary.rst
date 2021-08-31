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

Camera Pose:
  In TeleSculptor, a camera pose
  (often shortened to "camera")
  refers primarily to the model
  which describes the properties of a camera pose,
  including attributes such as focal length
  and world position and orientation.
  A frame may have at most one camera pose,
  which is unique to that frame
  (that is, camera poses are never shared)
  In the GUI, cameras are represented as frustums.

  At present, TeleSculptor assumes that all poses
  have the same camera *intrinsics*;
  that is, that all frame images are captured
  using the same physical camera.
  Note that this includes the lens' focal length.

Feature:
  A feature is a location that corresponds to an "interesting" point,
  such as the corner of an object or other "notable" point.
  The term "feature points" typically refers to features detected in imagery.

Track:
  A track is a collection of correlated features;
  that is, detected feature points
  estimated to correspond to the same landmark.

Landmark:
  A landmark is an estimated world location of a "true" feature
  that is computed from a feature track.

Residual:
  A residual, in general, is the difference between an observed value and an
  estimated value\ [#er]_. In TeleSculptor, the observed value is typically a
  detected feature point, and the estimated value is a landmark.

.. TODO document GCP, CRP

----

.. [#er] https://en.wikipedia.org/wiki/Errors_and_residuals_in_statistics
