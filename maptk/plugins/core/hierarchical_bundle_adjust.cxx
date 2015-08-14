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
 * \brief implementation of vital::algo::hierarchical_bundle_adjust
 */

#include "hierarchical_bundle_adjust.h"

#include <algorithm>
#include <iostream>
#include <limits>
#include <map>
#include <vector>

#include <math.h>

#include <boost/foreach.hpp>
#include <boost/timer/timer.hpp>

#include <vital/algo/optimize_cameras.h>
#include <vital/algo/triangulate_landmarks.h>
#include <maptk/metrics.h>

#include <vital/types/camera.h>
#include <vital/exceptions.h>
#include <vital/vital_types.h>

using namespace kwiver::vital;

namespace kwiver {
namespace maptk {

namespace core
{

namespace // anonymous
{

/// subsample a every Nth camera
/**
 * Subsamples are chosen based on camera order index instead of frame nubmer,
 * as cameras given may not be in sequential order.
 *
 * The first camera in the map is given index 0 and the last given index
 * (cameras.size() - 1).
 */
camera_map::map_camera_t
subsample_cameras(camera_map::map_camera_t const& cameras, unsigned n)
{
  boost::timer::auto_cpu_timer t("Camera sub-sampling: %t sec CPU, %w sec wall\n");

  // if sub-sample is 1, no sub-sampling occurs, just return a copy of the map
  if (n == 1)
  {
    return cameras;
  }

  camera_map::map_camera_t subsample;
  unsigned int i = 0;
  BOOST_FOREACH(camera_map::map_camera_t::value_type const& p, cameras)
  {
    if (i % n == 0)
    {
      subsample[p.first] = p.second;
    }
    ++i;
  }
  return subsample;
}

/// Integer interpolation -- used with indices, so can assume positive
frame_id_t
int_interp(frame_id_t a, frame_id_t b, double p)
{
  // intend for static cast acts as floor in rounding.
  return static_cast<frame_id_t>(a*(1.0-p) + b*p + 0.5);
}

} // end anonymous namespace


/// private implementation / data container for hierarchical_bundle_adjust
class hierarchical_bundle_adjust::priv
{
public:
  priv()
    : initial_sub_sample(1)
    , interpolation_rate(0)
    , rmse_reporting_enabled(false)
  {
  }

  priv(priv const& other)
    : initial_sub_sample(other.initial_sub_sample)
    , interpolation_rate(other.interpolation_rate)
    , sba(!other.sba ? algo::bundle_adjust_sptr()
                     : other.sba->clone())
    , camera_optimizer(!other.camera_optimizer ? algo::optimize_cameras_sptr()
                                               : other.camera_optimizer->clone())
    , lm_triangulator(!other.lm_triangulator ? algo::triangulate_landmarks_sptr()
                                             : other.lm_triangulator->clone())
  {
  }

  unsigned int initial_sub_sample;
  unsigned int interpolation_rate;
  bool rmse_reporting_enabled;

  vital::algo::bundle_adjust_sptr sba;
  vital::algo::optimize_cameras_sptr camera_optimizer;
  vital::algo::triangulate_landmarks_sptr lm_triangulator;
};


/// Constructor
hierarchical_bundle_adjust
::hierarchical_bundle_adjust()
  : d_(new priv)
{ }


/// Copy constructor
hierarchical_bundle_adjust
::hierarchical_bundle_adjust(hierarchical_bundle_adjust const& other)
  : d_(new priv(*other.d_))
{
}


/// Destructor
hierarchical_bundle_adjust
::~hierarchical_bundle_adjust() MAPTK_NOTHROW
{
}


/// Get this algorithm's \link maptk::kwiver::config_block configuration block \endlink
  vital::config_block_sptr
hierarchical_bundle_adjust
::get_configuration() const
{
  vital::config_block_sptr config = vital::algo::bundle_adjust::get_configuration();

  config->set_value("initial_sub_sample", d_->initial_sub_sample,
                    "Sub-sample the given cameras by this factor. Gaps will "
                    "then be filled in by iterations of interpolation.");

  config->set_value("interpolation_rate", d_->interpolation_rate,
                    "Number of cameras to fill in each iteration. When this "
                    "is set to 0, we will interpolate all missing cameras "
                    "at the first moment possible.");

  config->set_value("enable_rmse_reporting", d_->rmse_reporting_enabled,
                    "Enable the reporting of RMSE statistics at various "
                    "stages of this algorithm. Constant calculating of RMSE "
                    "may effect run time of the algorithm.");

  vital::algo::bundle_adjust::get_nested_algo_configuration(
      "sba_impl", config, d_->sba
      );
  vital::algo::optimize_cameras::get_nested_algo_configuration(
      "camera_optimizer", config, d_->camera_optimizer
      );
  vital::algo::triangulate_landmarks::get_nested_algo_configuration(
      "lm_triangulator", config, d_->lm_triangulator
      );

  return config;
}


/// Set this algorithm's properties via a config block
void
hierarchical_bundle_adjust
::set_configuration(vital::config_block_sptr config)
{
  d_->initial_sub_sample = config->get_value<unsigned int>("initial_sub_sample", d_->initial_sub_sample);
  d_->interpolation_rate = config->get_value<unsigned int>("interpolation_rate", d_->interpolation_rate);
  d_->rmse_reporting_enabled = config->get_value<bool>("enable_rmse_reporting", d_->rmse_reporting_enabled);

  vital::algo::bundle_adjust::set_nested_algo_configuration(
      "sba_impl", config, d_->sba
      );
  vital::algo::optimize_cameras::set_nested_algo_configuration(
      "camera_optimizer", config, d_->camera_optimizer
      );
  vital::algo::triangulate_landmarks::set_nested_algo_configuration(
      "lm_triangulator", config, d_->lm_triangulator
      );
}


/// Check that the algorithm's configuration vital::config_block is valid
bool
hierarchical_bundle_adjust
::check_configuration(vital::config_block_sptr config) const
{
  bool valid = true;

#define MAPTK_HSBA_CHECK_FAIL(msg) \
  std::cerr << "MAPTK HSBA CONFIG CHECK FAIL: " << msg << std::endl; \
  valid = false

  // using long to allow negatives and maintain numerical capacity of
  // unsigned int as the values would otherwise be stored as.
  if (config->has_value("initial_sub_sample")
      && config->get_value<long>("initial_sub_sample") <= 0)
  {
    MAPTK_HSBA_CHECK_FAIL("\"initial_sub_sample\" must be greater than 0. Given: "
                          << config->get_value<long>("initial_sub_sample"));
  }
  if (config->has_value("interpolation_rate")
      && config->get_value<long>("interpolation_rate") < 0)
  {
    MAPTK_HSBA_CHECK_FAIL("\"interpolation_rate\" must be >= 0. Given: "
                          << config->get_value<long>("interpolation_rate"));
  }

  if (!vital::algo::bundle_adjust::check_nested_algo_configuration("sba_impl", config))
  {
    MAPTK_HSBA_CHECK_FAIL("sba_impl configuration invalid.");
  }

  if (config->get_value<std::string>("camera_optimizer:type", "") == "")
  {
    std::cerr << "HSBA per-iteration camera optimization disabled" << std::endl;
  }
  else if (!vital::algo::optimize_cameras::check_nested_algo_configuration("camera_optimizer", config))
  {
    MAPTK_HSBA_CHECK_FAIL("camera_optimizer configuration invalid.");
  }

  if (config->get_value<std::string>("lm_triangulator:type", "") == "")
  {
    std::cerr << "HSBA per-iteration LM Triangulation disabled" << std::endl;
  }
  else if (!vital::algo::triangulate_landmarks::check_nested_algo_configuration("lm_triangulator", config))
  {
    std::cerr << "lm_triangulator type: \"" << config->get_value<std::string>("lm_triangulator:type") << "\"" << std::endl;
    MAPTK_HSBA_CHECK_FAIL("lm_triangulator configuration invalid.");
  }

#undef MAPTK_HSBA_CHECK_FAIL

  // camera optimizer and lm triangulator are optional. If not set, pointers
  // will be 0.

  return valid;
}


/// Optimize the camera and landmark parameters given a set of tracks
/**
 * Making naive assuptions:
 *  - cameras we are given are in sequence (no previous sub-sampling and no frame gaps)
 *  - given camera map evenly interpolates with the current configuration
 *  - Assuming that all frames we interpolate have tracks/landmarks with which
 *    to optimize that camera over.
 */
void
hierarchical_bundle_adjust
::optimize(camera_map_sptr & cameras,
           landmark_map_sptr & landmarks,
           track_set_sptr tracks) const
{
  using namespace std;

  //frame_id_t orig_max_frame = cameras->cameras().rbegin()->first;
  cerr << "Cameras given to hsba: " << cameras->size() << endl;
  size_t num_orig_cams = cameras->size();

  // If interpolation rate is 0, then that means that all intermediate frames
  // should be interpolated on the first step. Due to how the algorithm
  // functions, set var to unsigned int max.
  frame_id_t ir = d_->interpolation_rate;
  if (ir == 0)
  {
    ir = std::numeric_limits<frame_id_t>::max();
  }
  cerr << "Interpolation rate: " << ir << endl;

  // Sub-sample cameras
  // Always adding the last camera (if not already in there) to the sub-
  // sampling in order to remove the complexity of interpolating into empty
  // space (constant operation).
  unsigned int ssr = d_->initial_sub_sample;
  camera_map::map_camera_t input_cams = cameras->cameras(),
                           acm;
  acm = subsample_cameras(input_cams, ssr);
  acm[input_cams.rbegin()->first] = input_cams.rbegin()->second;
  camera_map_sptr active_cam_map(new simple_camera_map(acm));
  cerr << "Subsampled cameras: " << active_cam_map->size() << endl;

  // need to have at least 2 cameras
  if (active_cam_map->size() < 2)
  {
    throw invalid_value("Camera map given is of insufficient length.");
  }

  cerr << "Entering loop..." << endl;
  bool done = false;
  do
  {
    cerr << "----------" << endl;

    cerr << "SBA optimizing active cameras (" << active_cam_map->size() << " cams)" << endl;
    // updated active_cam_map and landmarks
    { // scope block
      boost::timer::auto_cpu_timer t("inner-SBA iteration: %t sec CPU, %w sec wall\n");
      d_->sba->optimize(active_cam_map, landmarks, tracks);
    }

    double rmse = reprojection_rmse(active_cam_map->cameras(),
                                    landmarks->landmarks(),
                                    tracks->tracks());
    cerr << "--> SBA iteration RMSE: " << rmse << endl;

    // If we've just completed SBA with all original frames in the new map,
    // then we're done.
    cerr << "completion check: " << active_cam_map->size() << " == " << num_orig_cams << " --> ";
    if (active_cam_map->size() == num_orig_cams)
    {
      cerr << "yup" << endl;
      done = true;
    }

    // perform interpolation between frames that have gaps in between them
    else
    {
      cerr << "nope" << endl;

      camera_map::map_camera_t
        // separated interpolated camera storage
        interped_cams,
        // concrete map of current active cameras
        ac_map = active_cam_map->cameras();

      // pre-allocation of variables for performance
      size_t ir_l; // local interpolation rate as gaps available may be less than global rate
      double f;
      frame_id_t i2;
      frame_id_t cur_frm, next_frm;
      camera_sptr cur_cam, next_cam;

      // Iterate through frames and cameras, interpolating across gaps when found
      // ASSUMING even interpolation for now
      camera_map::map_camera_t::const_iterator it = ac_map.begin();
      { // scope block
        boost::timer::auto_cpu_timer t("interpolating cams: %t sec CPU, %w sec wall\n");
        while (it != ac_map.end())
        {
          cur_frm = it->first;
          cur_cam = it->second;
          ++it;

          // If we're not at the end of the active camera sequence
          if (it != ac_map.end())
          {
            next_frm = it->first;
            next_cam = it->second;
            //cerr << "Interpolating cam between frames " << cur_frm << " and " << next_frm << endl;

            // this specific gap's interpolation rate -- gap may be smaller than ir
            ir_l = std::min(ir, next_frm - cur_frm - 1);

            for (double i = 1; i <= ir_l; ++i)
            {
              // Determine the integer associated with the interpolation step,
              // then determine the fraction location of that integer between
              // the two end points.

              // absolute fraction, might not land on integer
              f = i / (ir_l + 1);

              // aproximate interpolation snapped to nearest integer
              i2 = int_interp(cur_frm, next_frm, f);

              // fraction position of interpoated integer
              f = static_cast<double>(i2 - cur_frm) / (next_frm - cur_frm);

              interped_cams[i2] = interpolate_camera(cur_cam, next_cam, f);
            }

          }
        }
      }
      camera_map_sptr interped_cams_p(new simple_camera_map(interped_cams));

      // Optimize new camers
      if (d_->camera_optimizer)
      {
        cerr << "Optimizing new interpolated cameras (" << interped_cams.size() << " cams)" << endl;
        if (d_->rmse_reporting_enabled)
        {
          cerr << "\t- pre-optimization RMSE : "
               << reprojection_rmse(interped_cams_p->cameras(),
                                    landmarks->landmarks(),
                                    tracks->tracks())
               << endl;
        }

        { // scope block
          boost::timer::auto_cpu_timer t("\t- cameras optimization: %t sec CPU, %w sec wall\n");
          d_->camera_optimizer->optimize(interped_cams_p, tracks, landmarks);
        }

        if (d_->rmse_reporting_enabled)
        {
          cerr << "\t- post-optimization RMSE: "
               << reprojection_rmse(interped_cams_p->cameras(),
                                    landmarks->landmarks(),
                                    tracks->tracks())
               << endl;
        }
      }
      // adding optimized interpolated cameras to the map of existing cameras
      BOOST_FOREACH(camera_map::map_camera_t::value_type const& p, interped_cams_p->cameras())
      {
        ac_map[p.first] = p.second;
      }
      // Create new sptr of modified ac_map
      cerr << "Re-assiging new active cam map sptr (" << active_cam_map->size() << " cams)" << endl;
      active_cam_map = camera_map_sptr(new simple_camera_map(ac_map));
      if (d_->rmse_reporting_enabled)
      {
        cerr << "\t- reprojection RMSE of combined map: "
             << reprojection_rmse(active_cam_map->cameras(),
                                  landmarks->landmarks(),
                                  tracks->tracks())
             << endl;
      }

      // LM triangulation
      if (d_->lm_triangulator)
      {
        cerr << "Triangulating landmarks after interpolating cameras" << endl;
        if (d_->rmse_reporting_enabled)
        {
          cerr << "\t- pre-triangulation RMSE : "
               << reprojection_rmse(active_cam_map->cameras(),
                                    landmarks->landmarks(),
                                    tracks->tracks())
               << endl;
        }

        { // scoped block
          boost::timer::auto_cpu_timer t("\t- lm triangulation: %t sec CPU, %w sec wall\n");
          d_->lm_triangulator->triangulate(active_cam_map, tracks, landmarks);
        }

        if (d_->rmse_reporting_enabled)
        {
          cerr << "\t- post-triangulation RMSE: "
               << reprojection_rmse(active_cam_map->cameras(),
                                    landmarks->landmarks(),
                                    tracks->tracks())
               << endl;
        }
      }

    }
  } while (!done);

  // push up the resultant cameras
  cameras = active_cam_map;
}


} // end namespace core

} // end namespace maptk
} // end namespace kwiver
