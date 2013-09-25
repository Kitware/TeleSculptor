/*ckwg +5
 * Copyright 2013 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#ifndef MAPTK_COVARIANCE_3D_H_
#define MAPTK_COVARIANCE_3D_H_

#include <vector>

namespace maptk
{

/// A representation of covariance of a measurement in 3D
class covariance_3d
{
public:
  /// Default Constructor
  covariance_3d()
  {
    s_[0] = s_[1] = s_[2] = 1.0;
    s_[3] = s_[4] = s_[5] = 0.0;
  }

  /// Constructor for a feature
  covariance_3d(double sxx, double syy, double szz,
                double sxy, double sxz, double syz)
  {
    this->set(sxx, syy, szz, sxy, sxz, sxy);
  }

  // Accessor for the variance in X
  double sxx() const { return s_[0]; }
  // Accessor for the variance in Y
  double syy() const { return s_[1]; }
  // Accessor for the variance in Z
  double szz() const { return s_[2]; }
  // Accessor for the covariance in X and Y
  double sxy() const { return s_[3]; }
  // Accessor for the covariance in X and Z
  double sxz() const { return s_[4]; }
  // Accessor for the covariance in Y and Z
  double syz() const { return s_[5]; }

  // Set the feature position in image space
  void set(double sxx, double syy, double szz,
           double sxy, double sxz, double syz) 
  { 
    s_[0] = sxx;
    s_[1] = syy;
    s_[2] = szz;
    s_[3] = sxy;
    s_[4] = sxz;
    s_[5] = syz;
  }

  /// Compute the generalized variance (determinant of covariance)
  double generalized_variance() const;

  /// Access the underlying data
  const double* data() const { return s_; }

protected:

  double s_[6];

};

/// output stream operator for a feature
vcl_ostream&  operator<<(vcl_ostream& s, const covarariance_3d& c);

/// input stream operator for a feature
vcl_istream&  operator>>(vcl_istream& s, covariance_3d& c);


} // end namespace maptk


#endif // MAPTK_COVARIANCE_3D_H_
