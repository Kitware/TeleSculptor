/*ckwg +5
 * Copyright 2013 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#ifndef MAPTK_LANDMARK_H_
#define MAPTK_LANDMARK_H_

#include <vector>

#include "covariance_3d.h"

namespace maptk
{

/// A representation of a 3D world point
class landmark
{
public:
  /// Default Constructor
  landmark();

  /// Constructor for a feature
  landmark(double x, double y, double z);

  // Accessor for the X world coordinate
  double x() const { return x_; }
  // Accessor for the Y world coordinate
  double y() const { return y_; }
  // Accessor for the Z world coordinate
  double z() const { return z_; }
  // Accessor for the feature scale
  double scale() const { return scale_; }
  // Accessor for the covariance
  const covariance_3d& covar() const { return covar_; }

  // Set the feature position in image space
  void set_pos(double x, double y, double z) 
  { 
    x_ = x; 
    y_ = y; 
    z_ = z;
  }
  /// Set the scale of the feature
  void set_scale(double scale) { scale_ = scale; }
  /// Set the covariance matrix of the feature
  void set_covar(const covariance_3d& covar) { covar_ = covar; }

protected:

  double x_;
  double y_;
  double z_;
  double scale_;
  covariance_3d covar_;
};

/// output stream operator for a landmark
vcl_ostream&  operator<<(vcl_ostream& s, const landmark& m);

/// input stream operator for a landmark
vcl_istream&  operator>>(vcl_istream& s, landmark& m);


} // end namespace maptk


#endif // MAPTK_LANDMARK_H_
