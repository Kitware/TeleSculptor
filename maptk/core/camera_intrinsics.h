/*ckwg +5
 * Copyright 2013 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#ifndef MAPTK_CAMERA_INTRINSICS_H_
#define MAPTK_CAMERA_INTRINSICS_H_

#include <iostream>

#include "vector.h"

namespace maptk
{

/// A representation of camera intrinsic parameters
template <typename T>
class camera_intrinsics_
{
public:
  /// Default Constructor
  camera_intrinsics_<T>();

  /// Constructor for camera intrinsics
  camera_intrinsics_<T>(T focal_length,
                        const vector_2_<T>& prin_pt,
                        T aspect_ratio=1.0,
                        T skew=0.0);

protected:
  T focal_length_;
  vector_2_<T> prin_pt_;
  T aspect_ratio_;
  T skew_;
};


typedef camera_intrinsics_<double> camera_intrinsics_d;
typedef camera_intrinsics_<float> camera_intrinsics_f;

/// output stream operator for camera intrinsics
template <typename T>
std::ostream&  operator<<(std::ostream& s, const camera_intrinsics_<T>& k);

/// input stream operator for camera intrinsics
template <typename T>
std::istream&  operator>>(std::istream& s, camera_intrinsics_<T>& c);


} // end namespace maptk


#endif // MAPTK_CAMERA_INTRINSICS_H_
