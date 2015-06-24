/*ckwg +29
 * Copyright 2015 by Kitware, Inc.
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
 * \brief Implementation of Ceres bundle adjustment algorithm
 */

#include "bundle_adjust.h"

#include <iostream>
#include <set>

#include <boost/foreach.hpp>
#include <boost/timer/timer.hpp>

#include <maptk/eigen_io.h>
#include <maptk/plugins/ceres/reprojection_error.h>

#include <ceres/ceres.h>

using boost::timer::cpu_times;
using boost::timer::nanosecond_type;


namespace maptk
{

namespace ceres
{


/// Private implementation class
class bundle_adjust::priv
{
public:
  /// Constructor
  priv()
  : verbose(false)
  {
  }

  priv(const priv& other)
  : verbose(other.verbose)
  {
  }

  /// the Ceres solver problem
  ::ceres::Problem problem;
  /// the Ceres solver options
  ::ceres::Solver::Options options;
  /// verbose output
  bool verbose;
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
  typedef maptk::camera_map::map_camera_t map_camera_t;
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
  map_camera_t cams = cameras->cameras();
  map_landmark_t lms = landmarks->landmarks();
  std::vector<track_sptr> trks = tracks->tracks();

  // Extract the landmark locations into a mutable map
  typedef std::map<track_id_t, std::vector<double> > lm_param_map_t;
  lm_param_map_t landmark_params;
  BOOST_FOREACH(const map_landmark_t::value_type& lm, lms)
  {
    vector_3d loc = lm.second->loc();
    landmark_params[lm.first] = std::vector<double>(loc.data(), loc.data()+3);
  }

  // Extract the camera parameters into a mutable map
  typedef std::map<frame_id_t, std::vector<double> > cam_param_map_t;
  cam_param_map_t camera_params;
  std::vector<double> intrinsic_params(5);
  BOOST_FOREACH(const map_camera_t::value_type& c, cams)
  {
    vector_3d center = c.second->center();
    std::vector<double> params(6);
    std::copy(center.data(), center.data()+3, params.begin());
    vector_3d rot = c.second->rotation().rodrigues();
    std::copy(rot.data(), rot.data()+3, params.begin()+3);
    camera_intrinsics_d K = c.second->intrinsics();
    camera_params[c.first] = params;

    intrinsic_params[0] = K.focal_length();
    intrinsic_params[1] = K.principal_point().x();
    intrinsic_params[2] = K.principal_point().y();
    intrinsic_params[3] = K.aspect_ratio();
    intrinsic_params[4] = K.skew();
  }

  // Add the residuals for each relevant observation
  BOOST_FOREACH(const track_sptr& t, trks)
  {
    const track_id_t id = t->id();
    lm_param_map_t::iterator lm_itr = landmark_params.find(id);
    // skip this track if the landmark is not in the set to optimize
    if( lm_itr == landmark_params.end() )
    {
      continue;
    }

    for(track::history_const_itr ts = t->begin(); ts != t->end(); ++ts)
    {
      cam_param_map_t::iterator cam_itr = camera_params.find(ts->frame_id);
      if( cam_itr == camera_params.end() )
      {
        continue;
      }
      vector_2d pt = ts->feat->loc();
      d_->problem.AddResidualBlock(reprojection_error::create(pt.x(), pt.y()),
                                   NULL,
                                   &intrinsic_params[0],
                                   &cam_itr->second[0],
                                   &lm_itr->second[0]);
    }
  }

  d_->options.linear_solver_type = ::ceres::DENSE_SCHUR;
  d_->options.minimizer_progress_to_stdout = true;

  ::ceres::Solver::Summary summary;
  ::ceres::Solve(d_->options, &d_->problem, &summary);
  std::cout << summary.FullReport() << "\n";

  // Update the landmarks with the optimized values
  BOOST_FOREACH(const lm_param_map_t::value_type& lmp, landmark_params)
  {
    vector_3d pos = Eigen::Map<const vector_3d>(&lmp.second[0]);
    lms[lmp.first] = landmark_sptr(new landmark_d(pos));
  }

  // Update the cameras with the optimized values
  BOOST_FOREACH(const cam_param_map_t::value_type& cp, camera_params)
  {
    vector_3d center = Eigen::Map<const vector_3d>(&cp.second[0]);
    rotation_d rot(vector_3d(Eigen::Map<const vector_3d>(&cp.second[3])));

    camera_intrinsics_d K = cams[cp.first]->intrinsics();
    K.set_focal_length(intrinsic_params[0]);
    vector_2d pp((Eigen::Map<const vector_2d>(&intrinsic_params[1])));
    K.set_principal_point(pp);
    K.set_aspect_ratio(intrinsic_params[3]);
    K.set_skew(intrinsic_params[4]);
    cams[cp.first] = camera_sptr(new camera_d(center, rot, K));
  }


  cameras = camera_map_sptr(new simple_camera_map(cams));
  landmarks = landmark_map_sptr(new simple_landmark_map(lms));

#undef MAPTK_SBA_TIMED
}


} // end namespace ceres

} // end namespace maptk
