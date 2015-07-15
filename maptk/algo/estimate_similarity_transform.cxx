/*ckwg +29
 * Copyright 2013-2015 by Kitware, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither name of Kitware, Inc. nor the names of any contributors may be used
 *    to endorse or promote products derived from this software without specific
 *    prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS ``AS IS''
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHORS OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * \file
 * \brief Implementation of wrapper functions in similarity transform
 *        estimation algorithm definition.
 */

#include <boost/foreach.hpp>

#include <vital/algo/algorithm.txx>
#include <maptk/algo/estimate_similarity_transform.h>


/// \cond DoxygenSuppress
INSTANTIATE_ALGORITHM_DEF(maptk::algo::estimate_similarity_transform);
/// \endcond


namespace maptk
{

namespace algo
{

using namespace kwiver::vital;

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
 * \tparam M      Map type whose value_type::second_type is a boost::shared_ptr
 * \tparam afunc  Pointer to the accessor function in the object that is
 *                contained in the boost::shared_ptr.
 *
 * \param from_map      map of type M of objects at \c from position
 * \param to_map        map of type M of objects at \c to position
 * \param from_pts      vector in which to store \c from points that have
 *                      a corresponding \c to point.
 * \param to_pts        vector in which to store \c to points that have
 *                      a corresponding \c from point.
 */
template< typename M,
          vector_3d (M::value_type::second_type::element_type::*afunc)() const >
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
  map_to_pts< camera_map::map_camera_t, &camera::center >
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
  map_to_pts< landmark_map::map_landmark_t, &landmark::loc >
    (from_map, to_map, from_pts, to_pts);
  return this->estimate_transform(from_pts, to_pts);
}


#undef MAPTK_EST_MAP_TO_PTS


} // end namespace algo

} // end namespace maptk
