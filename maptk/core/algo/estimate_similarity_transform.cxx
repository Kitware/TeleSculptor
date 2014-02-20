/*ckwg +5
 * Copyright 2013-2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

/**
 * \file
 * \brief Implementation of wrapper functions in similarity transform
 *        estimation algorithm definition.
 */

#include <boost/foreach.hpp>
#include <maptk/core/algo/algorithm.txx>
#include <maptk/core/algo/estimate_similarity_transform.h>


INSTANTIATE_ALGORITHM_DEF(maptk::algo::estimate_similarity_transform);


namespace maptk
{

namespace algo
{


/// Estimate the similarity transform between two parallel sets of cameras
similarity_d
estimate_similarity_transform
::estimate_transform(std::vector<camera_sptr> &from,
                     std::vector<camera_sptr> &to) const
{
  std::vector<vector_3d> from_pts, to_pts;
  BOOST_FOREACH(camera_sptr c, from)
  {
    from_pts.push_back(c->center());
  }
  BOOST_FOREACH(camera_sptr c, to)
  {
    to_pts.push_back(c->center());
  }
  return this->estimate_transform(from_pts, to_pts);
}


/// Estimate the similarity transform between two parallel sets of landmarks.
similarity_d
estimate_similarity_transform
::estimate_transform(std::vector<landmark_sptr> &from,
                     std::vector<landmark_sptr> &to) const
{
  std::vector<vector_3d> from_pts, to_pts;
  BOOST_FOREACH(landmark_sptr l, from)
  {
    from_pts.push_back(l->loc());
  }
  BOOST_FOREACH(landmark_sptr l, to)
  {
    to_pts.push_back(l->loc());
  }
  return this->estimate_transform(from_pts, to_pts);
}


} // end namespace algo

} // end namespace maptk
