/*ckwg +5
 * Copyright 2013 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#ifndef MAPTK_COVARIANCE_2D_H_
#define MAPTK_COVARIANCE_2D_H_

#include <vector>

namespace maptk
{

/// A representation of covariance of a measurement in 2D
class covariance_2d
{
public:
  /// Default Constructor
  covariance_2d()
  {
    s_[0] = s_[1] = 1.0;
    s_[2] = 0.0;
  }

  /// Constructor for a feature
  covariance_2d(double sxx, double syy, double sxy)
  {
    s_[0] = sxx;
    s_[1] = syy;
    s_[2] = sxy;
  }

  // Accessor for the variance in X
  double sxx() const { return s_[0]; }
  // Accessor for the variance in Y
  double syy() const { return s_[1]; }
  // Accessor for the covariance in X and Y
  double sxy() const { return s_[2]; }

  // Set the feature position in image space
  void set(double sxx, double syy, double sxy) 
  { 
    s_[0] = sxx;
    s_[1] = syy;
    s_[2] = sxy;
  }

  /// Compute the generalized variance (determinant of covariance)
  double generalized_variance() const;

  /// Access the underlying data
  const double* data() const { return s_; }

protected:

  double s_[3];

};

/// output stream operator for a feature
vcl_ostream&  operator<<(vcl_ostream& s, const covarariance_2d& c);

/// input stream operator for a feature
vcl_istream&  operator>>(vcl_istream& s, covariance_2d& c);


} // end namespace maptk


#endif // MAPTK_COVARIANCE_2D_H_
