/*ckwg +5
 * Copyright 2013 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#ifndef MAPTK_CAMERA_H_
#define MAPTK_CAMERA_H_

#include <iostream>

#include "camera_intrinsics.h"
#include "covariance.h"
#include "rotation.h"
#include "vector.h"

namespace maptk
{

/// A representation of a camera
/// Contains camera location, orientation, and intrinsics
template <typename T>
class camera_
{
public:
  /// Default Constructor
  camera_<T>()
  : center_(T(0), T(0), T(0)),
    orientation_(),
    intrinsics_()
  {}

  /// Constructor - from camera center, rotation, and intrinsics
  camera_<T>(const vector_3_<T>& center,
             const rotation_<T>& rotation,
             const camera_intrinsics_<T>& intrincs = camera_intrinsics_<T>());

  /// Accessor for the camera center of projection (position)
  const vector_3_<T> center() const { return center_; }
  /// Accessor for the translation vector
  const vector_3_<T> translation() const { return - (orientation_ * center_); }
  /// Accessor for the covariance of camera center
  const covariance_<3,T>& center_covar() const { return center_covar_; }
  /// Accessor for the rotation
  const rotation_<T>& rotation() const { return orientation_; }
  /// Accessor for the intrinsics
  const camera_intrinsics_<T>& intrinsics() const { return intrinsics_; }

  /// Set the camera center of projection
  void set_center(const vector_3_<T>& center) { center_ = center; }
  /// Set the translation vector (relative to current rotation)
  void set_translation(const vector_3_<T>& translation)
  {
    center_ = - (orientation_.inverse() * translation);
  }
  /// Set the covariance matrix of the feature
  void set_center_covar(const covariance_<3,T>& center_covar) { center_covar_ = center_covar; }
  /// Set the rotation
  void set_rotation(const rotation_<T>& rotation) { orientation_ = rotation; }
  /// Set the intrinsics
  void set_intrinsics(const camera_intrinsics_<T>& intrinsics) { intrinsics_ = intrinsics; }

protected:
  /// The camera center of project
  vector_3_<T> center_;
  /// The covariance of the camera center location
  covariance_<3,T> center_covar_;
  /// The camera rotation
  rotation_<T> orientation_;
  /// The camera intrinics
  camera_intrinsics_<T> intrinsics_;
};


typedef camera_<double> camera_d;
typedef camera_<float> camera_f;


/// output stream operator for a camera
template <typename T>
std::ostream&  operator<<(std::ostream& s, const camera_<T>& c);

/// input stream operator for a camera
template <typename T>
std::istream&  operator>>(std::istream& s, camera_<T>& c);


} // end namespace maptk


#endif // MAPTK_CAMERA_H_
