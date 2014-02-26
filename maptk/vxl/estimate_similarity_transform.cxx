/*ckwg +5
 * Copyright 2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

/**
 * \file
 * \brief Implementation of VXL similarity transform estimation
 */

#include "estimate_similarity_transform.h"
#include <maptk/core/exceptions/algorithm.h>
#include <maptk/core/rotation.h>

#include <sstream>

#include <vcl_vector.h>
#include <vgl/algo/vgl_compute_similarity_3d.h>
#include <vgl/vgl_point_3d.h>
#include <vgl/vgl_vector_3d.h>
#include <vnl/vnl_quaternion.h>


namespace maptk
{

namespace vxl
{


/// Estimate the similarity transform between two corresponding point sets
similarity_d
estimate_similarity_transform
::estimate_transform(std::vector<vector_3d> const& from,
                     std::vector<vector_3d> const& to) const
{
  if (from.size() != to.size())
  {
    std::ostringstream sstr;
    sstr << "from and to sets for similarity estimation are not of equivalent "
         << "size! (from: " << from.size() << ", to: " << to.size() << ")";
    throw algorithm_exception(this->type_name(), this->impl_name(),
                              sstr.str());
  }
  // TODO: Determine why there is a deficiency with 3 correspondences
  else if (from.size() < 4)
  {
    std::ostringstream sstr;
    sstr << "At least 4 or more point pairs must be give in order to estimate "
         << "the similarity transformation. Given: " << from.size();
    throw algorithm_exception(this->type_name(), this->impl_name(), sstr.str());
  }

  // TODO: Test for collinearity
  // a <- (from[0], to[0])
  // b <- (from[1], to[1])
  // e <- some epsillon value
  // collinear <- true
  // for c in [(from[2], to[2]), ... , (from[n-1], to[n-1])] {
  //     if cross((b-a), (c-a)).magnitude > e {
  //        collinear <- false
  //        break
  //     }
  // }
  // if collinear {
  //    raise exception
  // }

  vgl_compute_similarity_3d<double> v_estimator;

  // Add point pairs to estimator object
  std::vector<vector_3d>::const_iterator from_iter, to_iter;
  for (from_iter = from.begin(), to_iter = to.begin();
       (from_iter != from.end()) && (to_iter != to.end());
       ++from_iter, ++to_iter)
  {
    v_estimator.add_points(
        vgl_point_3d<double>(from_iter->x(), from_iter->y(), from_iter->z()),
        vgl_point_3d<double>(to_iter->x(), to_iter->y(), to_iter->z())
    );
  }

  // Θ(N), where N = pts.size()
  v_estimator.estimate();

  // extract components
  vnl_quaternion<double> const& v_quat = v_estimator.rotation().as_quaternion();
  vgl_vector_3d<double> const& v_trans = v_estimator.translation();

  rotation_d const m_rot(vector_4d(v_quat.x(), v_quat.y(), v_quat.z(), v_quat.r()));
  vector_3d const m_trans(v_trans.x(), v_trans.y(), v_trans.z());

  return similarity_d(v_estimator.scale(), m_rot, m_trans);
}


} // end namespace vxl

} // end namespace maptk
