/*ckwg +5
 * Copyright 2013 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#ifndef MAPTK_LANDMARK_H_
#define MAPTK_LANDMARK_H_

#include <iostream>

#include "point_3d.h"
#include "covariance_3d.h"

namespace maptk
{

/// A representation of a 3D world point
template <typename T>
class landmark_
{
public:
  /// Default Constructor
  landmark_<T>();

  /// Constructor for a feature
  landmark_<T>(const point_3_<T>& loc);

  // Accessor for the world coordinates
  const point_3_<T>& loc() const { return loc_; }
  // Accessor for the feature scale
  T scale() const { return scale_; }
  // Accessor for the covariance
  const covariance_3d& covar() const { return covar_; }

  // Set the feature position in image space
  void set_loc(const point_3_<T>& loc) { loc_ = loc; }
  /// Set the scale of the feature
  void set_scale(T scale) { scale_ = scale; }
  /// Set the covariance matrix of the feature
  void set_covar(const covariance_3d& covar) { covar_ = covar; }

protected:

  point_3_<T> loc_;
  T scale_;
  covariance_3d covar_;
};

typedef landmark_<double> landmark_d;
typedef landmark_<float> landmark_f;

/// output stream operator for a landmark
template <typename T>
std::ostream&  operator<<(std::ostream& s, const landmark_<T>& m);

/// input stream operator for a landmark
template <typename T>
std::istream&  operator>>(std::istream& s, landmark_<T>& m);


} // end namespace maptk


#endif // MAPTK_LANDMARK_H_
