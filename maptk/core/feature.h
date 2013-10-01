/*ckwg +5
 * Copyright 2013 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#ifndef MAPTK_FEATURE_H_
#define MAPTK_FEATURE_H_

#include <iostream>

#include "vector.h"
#include "covariance_2d.h"

namespace maptk
{

/// A representation of a 2D image feature point.
template <typename T>
class feature_
{
public:
  /// Default Constructor
  feature_<T>();

  /// Constructor for a feature
  feature_<T>(const vector_2_<T>& loc, T mag=0.0,
              T scale=1.0, T angle=0.0);

  // Accessor for the image coordinates
  const vector_2_<T>& loc() const { return loc_; }
  // Accessor for the feature magnitude
  T magnitude() const { return magnitude_; }
  // Accessor for the feature scale
  T scale() const { return scale_; }
  // Accessor for the feature angle
  T angle() const { return angle_; }
  // Accessor for the covariance
  const covariance_2_<T>& covar() const { return covar_; }

  // Set the feature position in image space
  void set_loc(const vector_2_<T>& loc) { loc_ = loc; }
  /// Set the magnitude of the feature response
  void set_magnitude(T magnitude) { magnitude_ = magnitude; }
  /// Set the scale of the feature
  void set_scale(T scale) { scale_ = scale; }
  /// Set the angle of the feature
  void set_angle(T angle) { angle_ = angle; }
  /// Set the covariance matrix of the feature
  void set_covar(const covariance_2_<T>& covar) { covar_ = covar; }

protected:

  vector_2_<T> loc_;
  T magnitude_;
  T scale_;
  T angle_;
  covariance_2_<T> covar_;
};

typedef feature_<double> feature_d;
typedef feature_<float> feature_f;

/// output stream operator for a feature
template <typename T>
std::ostream&  operator<<(std::ostream& s, const feature_<T>& f);

/// input stream operator for a feature
template <typename T>
std::istream&  operator>>(std::istream& s, feature_<T>& f);


} // end namespace maptk


#endif // MAPTK_FEATURE_H_
