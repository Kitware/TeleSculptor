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


/// Helper macro for assigning camera/landmark map contents to point vectors
/**
 * \param map_t         std::map type to be used
 * \param from_map      map of objects at \c from position
 * \param to_map        map of objects at \c to position
 * \param from_pts      vector of vector_3d to store \c from points that have
 *                      a corresponding \c to point.
 * \param to_pts        vector of vector_3d to store \c to points that have
 *                      a corresponding \c from point.
 * \param accessor_func The name of the vector_3d point accessor function for
 *                      the \c map_t provided (i.e. \c loc for \c landmark
 *                      class or \c center for \c camera class)
 */
#define map_to_pts(map_t, from_map, to_map, from_pts, to_pts, accessor_func)  \
  do                                                                          \
  {                                                                           \
    map_t::const_iterator from_it = from_map.begin(),                         \
                          to_it   = to_map.begin();                           \
    /* STL map structure's stored data is ordered (binary search tree impl */ \
    /* O(from.size + to.size) */                                              \
    while (from_it != from_map.end() || to_it != to_map.end())                \
    {                                                                         \
      /* increment the lesser of the two when the frame IDs don't match */    \
      if (from_it->first > to_it->first)                                      \
        ++to_it;                                                              \
      else if (from_it->first < to_it->first)                                 \
        ++from_it;                                                            \
      else                                                                    \
      {                                                                       \
        from_pts.push_back(from_it->second->accessor_func());                 \
        to_pts.push_back(to_it->second->accessor_func());                     \
        ++from_it;                                                            \
        ++to_it;                                                              \
      }                                                                       \
    }                                                                         \
  } while (false)


/// Estimate the similarity transform between two corresponding camera maps
similarity_d
estimate_similarity_transform
::estimate_transform(camera_map const& from,
                     camera_map const& to) const
{
  // determine point pairings based on shared frame IDs
  std::vector<vector_3d> from_pts, to_pts;
  map_to_pts(camera_map::map_camera_t, from.cameras(), to.cameras(), from_pts, to_pts, center);
  return this->estimate_transform(from_pts, to_pts);
}


/// Estimate the similarity transform between two corresponding landmark maps
similarity_d
estimate_similarity_transform
::estimate_transform(landmark_map const& from,
                     landmark_map const& to) const
{
  // determine point pairings based on shared frame IDs
  std::vector<vector_3d> from_pts, to_pts;
  map_to_pts(landmark_map::map_landmark_t, from.landmarks(), to.landmarks(), from_pts, to_pts, loc);
  return this->estimate_transform(from_pts, to_pts);
}


#undef map_to_pts


} // end namespace algo

} // end namespace maptk
