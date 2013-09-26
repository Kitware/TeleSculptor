/*ckwg +5
 * Copyright 2013 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#ifndef MAPTK_POINT_3D_H_
#define MAPTK_POINT_3D_H_

#include <iostream>

namespace maptk
{

/// A representation of a 3D point.
template <typename T>
class point_3_
{
public:
  /// Default Constructor
  point_3_<T>() { d_[0] = d_[1] = d_[2] = 0.0; }

  /// Constructor for a 2D point
  point_3_<T>(T x, T y, T z) { this->set(x,y,z); }

  /// Accessor for the X coordinate
  T x() const { return d_[0]; }
  /// Accessor for the Y coordinate
  T y() const { return d_[1]; }
  /// Accessor for the Z coordinate
  T z() const { return d_[2]; }

  /// Set the feature position in image space
  void set_pos(T x, T y, T z) 
  { 
    d_[0] = x; 
    d_[1] = y; 
    d_[2] = z;
  }

  const point_3_<T>& operator +=(const point_3_<T>& other)
  {
    this->d_[0] += other.d_[0];
    this->d_[1] += other.d_[1];
    this->d_[2] += other.d_[2];
    return &this;
  }

  /// Get a const pointer to the underlying data
  const T* data() const { return d_; }

protected:

  T d_[3];
};

typedef point_3_<double> point_3d;
typedef point_3_<float> point_3f;


/// output stream operator for a 3D Point
template <typename T>
std::ostream&  operator<<(std::ostream& s, const point_3_<T>& p);

/// input stream operator for a 3D Point
template <typename T>
std::istream&  operator>>(std::istream& s, point_3_<T>& p);


} // end namespace maptk


#endif // MAPTK_POINT_3D_H_
