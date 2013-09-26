/*ckwg +5
 * Copyright 2013 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#ifndef MAPTK_FEATURE_H_
#define MAPTK_FEATURE_H_

#include <vector>

#include "covariance_2d.h"

namespace maptk
{

/// A representation of a 2D image feature point.
class feature
{
public:
  /// Default Constructor
  feature();

  /// Constructor for a feature
  feature(double x, double y, double mag=0.0, 
          double scale=1.0, double angle=0.0);

  // Accessor for the horizontal image coordinate
  double x() const { return x_; }
  // Accessor for the vertical image coordinate
  double y() const { return y_; }
  // Accessor for the feature magnitude
  double magnitude() const { return magnitude_; }
  // Accessor for the feature scale
  double scale() const { return scale_; }
  // Accessor for the feature angle
  double angle() const { return angle_; }i
  // Accessor for the covariance
  const covariance_2d& covar() const { return covar_; }

  // Set the feature position in image space
  void set_pos(double x, double y) 
  { 
    x_ = x; 
    y_ = y; 
  }
  /// Set the magnitude of the feature response
  void set_magnitude(double magnitude) { magnitude_ = magnitude; }
  /// Set the scale of the feature
  void set_scale(double scale) { scale_ = scale; }
  /// Set the angle of the feature
  void set_angle(double angle) { angle_ = angle; }
  /// Set the covariance matrix of the feature
  void set_covar(const covariance_2d& covar) { covar_ = covar; }

protected:

  double x_;
  double y_;
  double magnitude_;
  double scale_;
  double angle_;
  covariance_2d covar_;
};

/// output stream operator for a feature
std::ostream&  operator<<(std::ostream& s, const feature& f);

/// input stream operator for a feature
std::istream&  operator>>(std::istream& s, feature& f);


} // end namespace maptk


#endif // MAPTK_FEATURE_H_
