/*ckwg +5
 * Copyright 2013 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#ifndef MAPTK_COVARIANCE_2D_H_
#define MAPTK_COVARIANCE_2D_H_

#include <iostream>

namespace maptk
{

/// A representation of covariance of a measurement in 2D
template <typename T>
class covariance_2_
{
public:
  /// Default Constructor
  covariance_2_<T>()
  {
    s_[0] = s_[1] = 1.0;
    s_[2] = 0.0;
  }

  /// Constructor for covariance
  covariance_2_<T>(T sxx, T syy, T sxy)
  {
    s_[0] = sxx;
    s_[1] = syy;
    s_[2] = sxy;
  }

  // Accessor for the variance in X
  T sxx() const { return s_[0]; }
  // Accessor for the variance in Y
  T syy() const { return s_[1]; }
  // Accessor for the covariance in X and Y
  T sxy() const { return s_[2]; }

  // Set the feature position in image space
  void set(T sxx, T syy, T sxy)
  {
    s_[0] = sxx;
    s_[1] = syy;
    s_[2] = sxy;
  }

  /// Compute the generalized variance (determinant of covariance)
  T generalized_variance() const;

  /// Access the underlying data
  const T* data() const { return s_; }

protected:

  T s_[3];

};

typedef covariance_2_<double> covariance_2d;
typedef covariance_2_<float> covariance_2f;

/// output stream operator for a feature
template <typename T>
std::ostream&  operator<<(std::ostream& s, const covariance_2_<T>& c);

/// input stream operator for a feature
template <typename T>
std::istream&  operator>>(std::istream& s, covariance_2_<T>& c);


} // end namespace maptk


#endif // MAPTK_COVARIANCE_2D_H_
