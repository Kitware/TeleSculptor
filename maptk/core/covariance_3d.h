/*ckwg +5
 * Copyright 2013 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#ifndef MAPTK_COVARIANCE_3D_H_
#define MAPTK_COVARIANCE_3D_H_

#include <iostream>

namespace maptk
{

/// A representation of covariance of a measurement in 3D
template <typename T>
class covariance_3_
{
public:
  /// Default Constructor
  covariance_3_<T>()
  {
    s_[0] = s_[1] = s_[2] = 1.0;
    s_[3] = s_[4] = s_[5] = 0.0;
  }

  /// Constructor for covariance
  covariance_3_<T>(T sxx, T syy, T szz,
                   T sxy, T sxz, T syz)
  {
    this->set(sxx, syy, szz, sxy, sxz, sxy);
  }

  // Accessor for the variance in X
  T sxx() const { return s_[0]; }
  // Accessor for the variance in Y
  T syy() const { return s_[1]; }
  // Accessor for the variance in Z
  T szz() const { return s_[2]; }
  // Accessor for the covariance in X and Y
  T sxy() const { return s_[3]; }
  // Accessor for the covariance in X and Z
  T sxz() const { return s_[4]; }
  // Accessor for the covariance in Y and Z
  T syz() const { return s_[5]; }

  // Set the feature position in image space
  void set(T sxx, T syy, T szz,
           T sxy, T sxz, T syz)
  {
    s_[0] = sxx;
    s_[1] = syy;
    s_[2] = szz;
    s_[3] = sxy;
    s_[4] = sxz;
    s_[5] = syz;
  }

  /// Compute the generalized variance (determinant of covariance)
  T generalized_variance() const;

  /// Access the underlying data
  const T* data() const { return s_; }

protected:

  T s_[6];

};

typedef covariance_3_<double> covariance_3d;
typedef covariance_3_<float> covariance_3f;

/// output stream operator for a feature
template <typename T>
std::ostream&  operator<<(std::ostream& s, const covariance_3_<T>& c);

/// input stream operator for a feature
template <typename T>
std::istream&  operator>>(std::istream& s, covariance_3_<T>& c);


} // end namespace maptk


#endif // MAPTK_COVARIANCE_3D_H_
