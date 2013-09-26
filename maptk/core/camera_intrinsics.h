/*ckwg +5
 * Copyright 2013 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#ifndef MAPTK_CAMERA_INTRINSICS_H_
#define MAPTK_CAMERA_INTRINSICS_H_

#include <iostream>

namespace maptk
{

/// A representation of camera intrinsic parameters
class camera_intrinsics
{
public:
  /// Default Constructor
  camera_intrinsics();

  /// Constructor for a feature
  camera(double focal_length, 
         double aspect_ratio=1.0, 
         double skew=0.0);

protected:

};

/// output stream operator for a camera
std::ostream&  operator<<(std::ostream& s, const camera_intrinsics& c);

/// input stream operator for a camera
std::istream&  operator>>(std::istream& s, camera_intrinsics& c);


} // end namespace maptk


#endif // MAPTK_CAMERA_INTRINSICS_H_
