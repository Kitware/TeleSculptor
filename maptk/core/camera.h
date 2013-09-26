/*ckwg +5
 * Copyright 2013 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#ifndef MAPTK_CAMERA_H_
#define MAPTK_CAMERA_H_

#include <vector>

#include "covariance_3d.h"

namespace maptk
{

/// A representation of a camera
/// Contains camera location, orientation, and intrinsics
class camera
{
public:
  /// Default Constructor
  camera();

  /// Constructor for a feature
  camera(double x, double y, double z);

  // Accessor for the covariance of 3D location
  const covariance_3d& loc_covar() const { return loc_covar_; }

  /// Set the covariance matrix of the feature
  void set_loc_covar(const covariance_3d& loc_covar) { loc_covar_ = loc_covar; }
  

protected:

  covariance_3d loc_covar_;
};

/// output stream operator for a camera
std::ostream&  operator<<(std::ostream& s, const camera& c);

/// input stream operator for a camera
std::istream&  operator>>(std::istream& s, camera& c);


} // end namespace maptk


#endif // MAPTK_CAMERA_H_
