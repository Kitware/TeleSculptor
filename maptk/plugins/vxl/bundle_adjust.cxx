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
 * \brief Implementation of VXL bundle adjustment algorithm
 */

#include "bundle_adjust.h"

#include <iostream>
#include <set>

#include <boost/foreach.hpp>
#include <boost/timer/timer.hpp>

#include <maptk/plugins/vxl/camera_map.h>
#include <maptk/eigen_io.h>

#include <vpgl/algo/vpgl_bundle_adjust.h>

using boost::timer::cpu_times;
using boost::timer::nanosecond_type;


namespace maptk
{

namespace vxl
{


/// Private implementation class
class bundle_adjust::priv
{
public:
  /// Constructor
  priv()
  : verbose(false),
    use_m_estimator(false),
    m_estimator_scale(1.0),
    estimate_focal_length(false),
    normalize_data(true),
    max_iterations(1000),
    x_tolerance(1e-8),
    g_tolerance(1e-8)
  {
  }

  priv(const priv& other)
  : verbose(other.verbose),
    use_m_estimator(other.use_m_estimator),
    m_estimator_scale(other.m_estimator_scale),
    estimate_focal_length(other.estimate_focal_length),
    normalize_data(other.normalize_data),
    max_iterations(other.max_iterations),
    x_tolerance(other.x_tolerance),
    g_tolerance(other.g_tolerance)
  {
  }

  /// the vxl sparse bundle adjustor
  vpgl_bundle_adjust ba;
  // vpgl_bundle_adjust does not currently allow accessors for parameters,
  // so we need to cache the parameters here.
  bool verbose;
  bool use_m_estimator;
  double m_estimator_scale;
  bool estimate_focal_length;
  bool normalize_data;
  unsigned max_iterations;
  double x_tolerance;
  double g_tolerance;
};


/// Constructor
bundle_adjust
::bundle_adjust()
: d_(new priv)
{
}


/// Copy Constructor
bundle_adjust
::bundle_adjust(const bundle_adjust& other)
: d_(new priv(*other.d_))
{
}


/// Destructor
bundle_adjust
::~bundle_adjust()
{
}


/// Get this algorithm's \link maptk::config_block configuration block \endlink
config_block_sptr
bundle_adjust
::get_configuration() const
{
  // get base config from base class
  config_block_sptr config = maptk::algo::bundle_adjust::get_configuration();
  config->set_value("verbose", d_->verbose,
                    "If true, write status messages to the terminal showing "
                    "optimization progress at each iteration");
  config->set_value("use_m_estimator", d_->use_m_estimator,
                    "If true, use a M-estimator for a robust loss function. "
                    "Currently only the Beaton-Tukey loss function is supported.");
  config->set_value("m_estimator_scale", d_->m_estimator_scale,
                    "The scale of the M-estimator, if enabled, in pixels. "
                    "Inlier landmarks should project to within this distance "
                    "from the feature point.");
  config->set_value("estimate_focal_length", d_->estimate_focal_length,
                    "If true, estimate a shared intrinsic focal length for all "
                    "cameras.  Warning: there is often a depth/focal length "
                    "ambiguity which can lead to long optimizations.");
  config->set_value("normalize_data", d_->normalize_data,
                    "Normalize the data for numerical stability. "
                    "There is no reason not enable this option, except "
                    "for testing purposes.");
  config->set_value("max_iterations", d_->max_iterations,
                    "Termination condition: maximum number of LM iterations");
  config->set_value("x_tolerance", d_->x_tolerance,
                    "Termination condition: Relative change is parameters. "
                    "Exit when (mag(delta_params) / mag(params) < x_tol).");
  config->set_value("g_tolerance", d_->g_tolerance,
                    "Termination condition: Maximum gradient magnitude. "
                    "Exit when (max(grad_params) < g_tol)");
  return config;
}


/// Set this algorithm's properties via a config block
void
bundle_adjust
::set_configuration(config_block_sptr in_config)
{
  // Starting with our generated config_block to ensure that assumed values are present
  // An alternative is to check for key presence before performing a get_value() call.
  config_block_sptr config = this->get_configuration();
  config->merge_config(in_config);

  d_->verbose = config->get_value<bool>("verbose",
                                        d_->verbose);
  d_->ba.set_verbose(d_->verbose);

  d_->use_m_estimator = config->get_value<bool>("use_m_estimator",
                                                d_->use_m_estimator);
  d_->ba.set_use_m_estimator(d_->use_m_estimator);

  d_->m_estimator_scale = config->get_value<double>("m_estimator_scale",
                                                    d_->m_estimator_scale);
  d_->ba.set_m_estimator_scale(d_->m_estimator_scale);

  d_->estimate_focal_length = config->get_value<bool>("estimate_focal_length",
                                                      d_->estimate_focal_length);
  d_->ba.set_self_calibrate(d_->estimate_focal_length);

  d_->normalize_data = config->get_value<bool>("normalize_data",
                                               d_->normalize_data);
  d_->ba.set_normalize_data(d_->normalize_data);

  d_->max_iterations = config->get_value<unsigned>("max_iterations",
                                                   d_->max_iterations);
  d_->ba.set_max_iterations(d_->max_iterations);

  d_->x_tolerance = config->get_value<double>("x_tolerance",
                                              d_->x_tolerance);
  d_->ba.set_x_tolerence(d_->x_tolerance);

  d_->g_tolerance = config->get_value<double>("g_tolerance",
                                              d_->g_tolerance);
  d_->ba.set_g_tolerence(d_->g_tolerance);
}


/// Check that the algorithm's currently configuration is valid
bool
bundle_adjust
::check_configuration(config_block_sptr config) const
{
  return true;
}


/// Optimize the camera and landmark parameters given a set of tracks
void
bundle_adjust
::optimize(camera_map_sptr& cameras,
           landmark_map_sptr& landmarks,
           track_set_sptr tracks) const
{
  if( !cameras || !landmarks || !tracks )
  {
    // TODO throw an exception for missing input data
    return;
  }
  typedef vxl::camera_map::map_vcam_t map_vcam_t;
  typedef maptk::landmark_map::map_landmark_t map_landmark_t;

#define MAPTK_SBA_TIMED(msg, code) \
  do \
  { \
    boost::timer::cpu_timer t; \
    if (d_->verbose) \
    { \
      std::cerr << msg << " ... " << std::endl; \
    } \
    code \
    if (d_->verbose) \
    { \
      cpu_times elapsed = t.elapsed(); \
      /* convert nanosecond to seconds */ \
      double secs = static_cast<double>(elapsed.system + elapsed.user) * 0.000000001; \
      std::cerr << "--> " << secs << " s CPU" << std::endl; \
    } \
  } while(false)

  // extract data from containers
  map_vcam_t vcams = camera_map_to_vpgl(*cameras);
  map_landmark_t lms = landmarks->landmarks();
  std::vector<track_sptr> trks = tracks->tracks();

  //
  // Find the set of all frame numbers containing a camera and track data
  //

  // convidience set of all lm IDs that the active cameras view
  std::set<track_id_t> lm_ids;

  // Nested relation of frame number to a map of track IDs to feature of the
  // track on that frame
  typedef std::map<track_id_t, feature_sptr> super_map_inner_t;
  typedef std::map<frame_id_t, super_map_inner_t> super_map_t;
  super_map_t frame2track2feature_map;

  MAPTK_SBA_TIMED("Constructing id-map and super-map",
    BOOST_FOREACH(const map_vcam_t::value_type& p, vcams)
    {
      const frame_id_t& frame = p.first;
      track_set_sptr ftracks = tracks->active_tracks(static_cast<int>(frame));
      if (! ftracks || ftracks->size() == 0)
      {
        continue;
      }
      super_map_inner_t frame_lm2feature_map;

      BOOST_FOREACH(const track_sptr& t, ftracks->tracks())
      {
        const track_id_t id = t->id();
        // make sure the track id has an associated landmark

        if( lms.find(id) != lms.end() )
        {
          frame_lm2feature_map[id] = t->find(frame)->feat;
          lm_ids.insert(id);
        }
      }

      if( !frame_lm2feature_map.empty() )
      {
        frame2track2feature_map[frame] = frame_lm2feature_map;
      }
    }
  );

  //
  // Create a compact set of data to optimize,
  // with mapping back to original indices
  //

  // -> landmark mappings
  std::vector<track_id_t> lm_id_index;
  std::map<track_id_t, frame_id_t> lm_id_reverse_map;
  std::vector<vgl_point_3d<double> > active_world_pts;
  // -> camera mappings
  std::vector<frame_id_t> cam_id_index;
  std::map<frame_id_t, frame_id_t> cam_id_reverse_map;
  std::vector<vpgl_perspective_camera<double> > active_vcams;

  MAPTK_SBA_TIMED("Creating index mappings",
    BOOST_FOREACH(const track_id_t& id, lm_ids)
    {
      lm_id_reverse_map[id] = static_cast<track_id_t>(lm_id_index.size());
      lm_id_index.push_back(id);
      vector_3d pt = lms[id]->loc();
      active_world_pts.push_back(vgl_point_3d<double>(pt.x(), pt.y(), pt.z()));
    }
    BOOST_FOREACH(const super_map_t::value_type& p, frame2track2feature_map)
    {
      cam_id_reverse_map[p.first] = static_cast<frame_id_t>(cam_id_index.size());
      cam_id_index.push_back(p.first);
      active_vcams.push_back(vcams[p.first]);
    }
  );

  // Construct the camera/landmark visibility matrix
  std::vector<std::vector<bool> >
      mask(active_vcams.size(),
           std::vector<bool>(active_world_pts.size(), false));
  // Analogous 2D matrix of the track state (feature) location for a given
  // camera/landmark pair
  std::vector<std::vector<feature_sptr> >
      feature_mask(active_vcams.size(),
                   std::vector<feature_sptr>(active_world_pts.size(), feature_sptr()));
  // compact location vector
  std::vector<vgl_point_2d<double> > image_pts;

  MAPTK_SBA_TIMED("Creating masks and point vector",
    BOOST_FOREACH(const super_map_t::value_type& p, frame2track2feature_map)
    {
      // p.first  -> frame ID
      // p.second -> super_map_inner_t
      const frame_id_t c_idx = cam_id_reverse_map[p.first];
      std::vector<bool>& mask_row = mask[c_idx];
      std::vector<feature_sptr>& fmask_row = feature_mask[c_idx];
      BOOST_FOREACH(const super_map_inner_t::value_type& q, p.second)
      {
        // q.first  -> lm ID
        // q.second -> feature_sptr
        mask_row[lm_id_reverse_map[q.first]] = true;
        fmask_row[lm_id_reverse_map[q.first]] = q.second;
      }
    }
    // Populate the vector of observations in the correct order using mask
    // matrices
    vector_2d t_loc;
    for (unsigned int i=0; i<active_vcams.size(); ++i)
    {
      for (unsigned int j=0; j<active_world_pts.size(); ++j)
      {
        if(mask[i][j])
        {
          t_loc = feature_mask[i][j]->loc();
          image_pts.push_back(vgl_point_2d<double>(t_loc.x(), t_loc.y()));
        }
      }
    }
  );

  // Run the vpgl bundle adjustment on the selected data
  MAPTK_SBA_TIMED("VXL bundle optimization",
    d_->ba.optimize(active_vcams, active_world_pts, image_pts, mask);
  );

  // map optimized results back into maptk structures
  MAPTK_SBA_TIMED("Mapping optimized results back to MAPTK structures",
    for(unsigned int i=0; i<active_vcams.size(); ++i)
    {
      vcams[cam_id_index[i]] = active_vcams[i];
    }
    for(unsigned int i=0; i<active_world_pts.size(); ++i)
    {
      const vgl_point_3d<double>& pt = active_world_pts[i];
      vector_3d loc(pt.x(), pt.y(), pt.z());
      landmark_sptr lm = lms[lm_id_index[i]];
      if( landmark_d* lmd = dynamic_cast<landmark_d*>(lm.get()) )
      {
        lmd->set_loc(loc);
      }
      else if( landmark_f* lmf = dynamic_cast<landmark_f*>(lm.get()) )
      {
        lmf->set_loc(vector_3f(loc));
      }
    }
    cameras = camera_map_sptr(new camera_map(vcams));
    landmarks = landmark_map_sptr(new simple_landmark_map(lms));
  );

#undef MAPTK_SBA_TIMED
}


} // end namespace vxl

} // end namespace maptk
