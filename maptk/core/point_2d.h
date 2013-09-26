/*ckwg +5
 * Copyright 2013 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#ifndef MAPTK_POINT_2D_H_
#define MAPTK_POINT_2D_H_

#include <iostream>

namespace maptk
{

/// A representation of a 2D point.
template <typename T>
class point_2_
{
public:
  /// Default Constructor
  point_2_<T>() { d_[0] = d_[1] = 0.0; }

  /// Constructor for a 2D point
  point_2_<T>(T x, T y) { this->set_pos(x,y); }

  /// Accessor for the X coordinate
  T x() const { return d_[0]; }
  /// Accessor for the Y coordinate
  T y() const { return d_[1]; }

  /// Set the feature position in image space
  void set_pos(T x, T y) 
  { 
    d_[0] = x; 
    d_[1] = y; 
  }

  const point_2_<T>& operator +=(const point_2_<T>& other)
  {
    this->d_[0] += other.d_[0];
    this->d_[1] += other.d_[1];
    return &this;
  }

  /// Get a const pointer to the underlying data
  const T* data() const { return d_; }

protected:

  T d_[2];
};

typedef point_2_<double> point_2d;
typedef point_2_<float> point_2f;

/// output stream operator for a 2D Point
template <typename T>
std::ostream&  operator<<(std::ostream& s, const point_2_<T>& p);

/// input stream operator for a 2D Point
template <typename T>
std::istream&  operator>>(std::istream& s, point_2_<T>& p);


} // end namespace maptk


#endif // MAPTK_POINT_2D_H_
