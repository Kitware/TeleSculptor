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


/// \cond DoxygenSuppress
INSTANTIATE_ALGORITHM_DEF(maptk::algo::estimate_similarity_transform);
/// \endcond


namespace maptk
{

namespace algo
{


/// Estimate the similarity transform between two corresponding sets of cameras
similarity_d
estimate_similarity_transform
::estimate_transform(std::vector<camera_sptr> const& from,
                     std::vector<camera_sptr> const& to) const
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


/// Estimate the similarity transform between two corresponding sets of landmarks.
similarity_d
estimate_similarity_transform
::estimate_transform(std::vector<landmark_sptr> const& from,
                     std::vector<landmark_sptr> const& to) const
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


namespace
{

/// Helper function for assigning camera/landmark map contents to point vectors
/**
 * \tparam M      Map type
 * \tparam T      Type of the element stored under the boost::shared_ptr
 * \tparam afunc  Pointer to the accessor function.
 *
 * \param from_map      map of objects at \c from position
 * \param to_map        map of objects at \c to position
 * \param from_pts      vector of vector_3d to store \c from points that have
 *                      a corresponding \c to point.
 * \param to_pts        vector of vector_3d to store \c to points that have
 *                      a corresponding \c from point.
 */
template<typename M, typename T, vector_3d (T::*afunc)() const>
void map_to_pts(M const& from_map, M const& to_map,
                std::vector<vector_3d> &from_pts, std::vector<vector_3d> &to_pts)
{
  typename M::const_iterator from_it = from_map.begin(),
                             to_it   = to_map.begin();
  // STL map structure's stored data is ordered (binary search tree impl
  // O(from.size + to.size)
  while (from_it != from_map.end() && to_it != to_map.end())
  {
    // increment the lesser of the two when the frame IDs don't match
    if (from_it->first > to_it->first)
    {
      ++to_it;
    }
    else if (from_it->first < to_it->first)
    {
      ++from_it;
    }
    else // equal
    {
      from_pts.push_back( ((*from_it->second).*afunc)() );
      to_pts.push_back( ((*to_it->second).*afunc)() );
      ++from_it; ++to_it;
    }
  }
}

}


/// Estimate the similarity transform between two corresponding camera maps
similarity_d
estimate_similarity_transform
::estimate_transform(camera_map_sptr const from,
                     camera_map_sptr const to) const
{
  // determine point pairings based on shared frame IDs
  std::vector<vector_3d> from_pts, to_pts;
  camera_map::map_camera_t from_map = from->cameras(),
                           to_map = to->cameras();
  map_to_pts< camera_map::map_camera_t, camera, &camera::center >
    (from_map, to_map, from_pts, to_pts);
  return this->estimate_transform(from_pts, to_pts);
}


/// Estimate the similarity transform between two corresponding landmark maps
similarity_d
estimate_similarity_transform
::estimate_transform(landmark_map_sptr const from,
                     landmark_map_sptr const to) const
{
  // determine point pairings based on shared frame IDs
  std::vector<vector_3d> from_pts, to_pts;
  landmark_map::map_landmark_t from_map = from->landmarks(),
                               to_map = to->landmarks();
  map_to_pts< landmark_map::map_landmark_t, landmark, &landmark::loc >
    (from_map, to_map, from_pts, to_pts);
  return this->estimate_transform(from_pts, to_pts);
}


#undef MAPTK_EST_MAP_TO_PTS


} // end namespace algo

} // end namespace maptk
