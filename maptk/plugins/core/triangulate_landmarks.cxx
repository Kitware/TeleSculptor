/*ckwg +29
 * Copyright 2014-2015 by Kitware, Inc.
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
 * \brief Implementation of core triangulate landmarks algorithm
 */

#include "triangulate_landmarks.h"

#include <set>

#include <vital/vital_foreach.h>
#include <vital/logger/logger.h>

#include <maptk/triangulate.h>


namespace kwiver {
namespace maptk {

namespace core
{


/// Private implementation class
class triangulate_landmarks::priv
{
public:
  /// Constructor
  priv()
    : homogeneous(false),
      m_logger( vital::get_logger( "triangulate_landmarks" ))
  {
  }

  priv(const priv& other)
    : homogeneous(other.homogeneous),
      m_logger( vital::get_logger( "triangulate_landmarks" ))
  {
  }

  /// use the homogeneous method for triangulation
  bool homogeneous;
  /// logger handle
  vital::logger_handle_t m_logger;
};


/// Constructor
triangulate_landmarks
::triangulate_landmarks()
: d_(new priv)
{
}


/// Copy Constructor
triangulate_landmarks
::triangulate_landmarks(const triangulate_landmarks& other)
: d_(new priv(*other.d_))
{
}


/// Destructor
triangulate_landmarks
::~triangulate_landmarks()
{
}


/// Get this alg's \link vital::config_block configuration block \endlink
vital::config_block_sptr
triangulate_landmarks
::get_configuration() const
{
  // get base config from base class
  vital::config_block_sptr config
    = vital::algo::triangulate_landmarks::get_configuration();

  // Bad frame detection parameters
  config->set_value("homogeneous", d_->homogeneous,
                    "Use the homogeneous method for triangulating points. "
                    "The homogeneous method can triangulate points at or near "
                    "infinity and discard them.");

  return config;
}


/// Set this algorithm's properties via a config block
void
triangulate_landmarks
::set_configuration(vital::config_block_sptr in_config)
{
  // Starting with our generated config_block to ensure that assumed values are present
  // An alternative is to check for key presence before performing a get_value() call.
  vital::config_block_sptr config = this->get_configuration();
  config->merge_config(in_config);

  // Settings for bad frame detection
  d_->homogeneous = config->get_value<bool>("homogeneous", d_->homogeneous);
}


/// Check that the algorithm's currently configuration is valid
bool
triangulate_landmarks
::check_configuration(vital::config_block_sptr config) const
{
  return true;
}


/// Triangulate the landmark locations given sets of cameras and tracks
void
triangulate_landmarks
::triangulate(vital::camera_map_sptr cameras,
              vital::track_set_sptr tracks,
              vital::landmark_map_sptr& landmarks) const
{
  using namespace kwiver;
  if( !cameras || !landmarks || !tracks )
  {
    // TODO throw an exception for missing input data
    return;
  }
  typedef vital::camera_map::map_camera_t map_camera_t;
  typedef vital::landmark_map::map_landmark_t map_landmark_t;

  // extract data from containers
  map_camera_t cams = cameras->cameras();
  map_landmark_t lms = landmarks->landmarks();
  std::vector<vital::track_sptr> trks = tracks->tracks();

  // build a track map by id
  typedef std::map<vital::track_id_t, vital::track_sptr> track_map_t;
  track_map_t track_map;
  VITAL_FOREACH(const vital::track_sptr& t, trks)
  {
    track_map[t->id()] = t;
  }

  // the set of landmark ids which failed to triangulate
  std::set<vital::landmark_id_t> failed_landmarks;

  map_landmark_t triangulated_lms;
  VITAL_FOREACH(const map_landmark_t::value_type& p, lms)
  {
    // get the corresponding track
    track_map_t::const_iterator t_itr = track_map.find(p.first);
    if (t_itr == track_map.end())
    {
      // there is no track for the provided landmark
      continue;
    }
    const vital::track& t = *t_itr->second;

    // extract the cameras and image points for this landmarks
    std::vector<vital::camera_d> lm_cams;
    std::vector<vital::vector_2d> lm_image_pts;

    for (vital::track::history_const_itr tsi = t.begin(); tsi != t.end(); ++tsi)
    {
      if (!tsi->feat)
      {
        // there is no valid feature for this track state
        continue;
      }
      map_camera_t::const_iterator c_itr = cams.find(tsi->frame_id);
      if (c_itr == cams.end())
      {
        // there is no camera for this track state.
        continue;
      }
      lm_cams.push_back(vital::camera_d(*c_itr->second));
      lm_image_pts.push_back(tsi->feat->loc());
    }

    // if we found at least two views of this landmark, triangulate
    if (lm_cams.size() > 1)
    {
      bool bad_triangulation = false;
      vital::vector_3d pt3d;
      if (d_->homogeneous)
      {
        vital::vector_4d pt4d = triangulate_homog(lm_cams, lm_image_pts);
        if (std::abs(pt4d[3]) < 1e-6)
        {
          bad_triangulation = true;
          failed_landmarks.insert(p.first);
        }
        pt3d = pt4d.segment(0,3) / pt4d[3];
      }
      else
      {
        pt3d = triangulate_inhomog(lm_cams, lm_image_pts);
      }
      VITAL_FOREACH(vital::camera_d const& cam, lm_cams)
      {
        if(cam.depth(pt3d) < 0.0)
        {
          bad_triangulation = true;
          failed_landmarks.insert(p.first);
          break;
        }
      }
      if( !bad_triangulation )
      {
        vital::landmark_d* lm = new vital::landmark_d(pt3d);
        triangulated_lms[p.first] = vital::landmark_sptr(lm);
      }
    }
  }
  if( !failed_landmarks.empty() )
  {
    LOG_WARN( d_->m_logger,
              "failed to triangulate " << failed_landmarks.size()
              << " of " << lms.size() << " landmarks");
  }
  landmarks = vital::landmark_map_sptr(new vital::simple_landmark_map(triangulated_lms));
}


} // end namespace core

} // end namespace maptk
} // end namespace kwiver
