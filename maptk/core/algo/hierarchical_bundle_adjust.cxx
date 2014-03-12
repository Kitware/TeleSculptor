/*ckwg +5
 * Copyright 2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

/**
 * \file
 * \brief implementation of maptk::algo::hierarchical_bundle_adjust
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

#include <maptk/core/algo/optimize_cameras.h>
#include <maptk/core/camera.h>
#include <maptk/core/metrics.h>
#include <maptk/core/exceptions.h>
#include <maptk/core/types.h>


namespace maptk
{

namespace algo
{

namespace // anonymous
{

/// subsample a every Nth camera
/**
 * Subsamples based off camera order index where the first camera in the map
 * has index 0 and the last has index n-1.
 */
camera_map::map_camera_t
subsample_cameras(camera_map::map_camera_t const& cameras, unsigned n)
{
  boost::timer::auto_cpu_timer t("Camera sub-sampling: %t sec CPU, %w sec wall\n");

  // if sub-sample is 1, no sub-sampling occurs, just return a copy
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

} // end anonymous namespace


/// private implementation / data container for hierarchical_bundle_adjust
class hierarchical_bundle_adjust::priv
{
public:
  priv()
    : initial_sub_sample(1)
    , interpolation_rate(0)
  {
  }

  priv(priv const& other)
    : initial_sub_sample(other.initial_sub_sample)
    , interpolation_rate(other.interpolation_rate)
  {
  }

  unsigned int initial_sub_sample;
  unsigned int interpolation_rate;

  bundle_adjust_sptr sba;
  optimize_cameras_sptr camera_optimizer;
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


/// Get this algorithm's \link maptk::config_block configuration block \endlink
config_block_sptr
hierarchical_bundle_adjust
::get_configuration() const
{
  config_block_sptr config = maptk::algo::bundle_adjust::get_configuration();

  config->set_value("initial_sub_sample", d_->initial_sub_sample,
                    "Sub-sample the given cameras by this factor. Gaps will "
                    "then be filled in by iterations of interpolation.");

  config->set_value("interpolation_rate", d_->interpolation_rate,
                    "Number of cameras to fill in each iteration. When this "
                    "is set to 0, we will interpolate all missing cameras "
                    "at the first moment possible.");

  maptk::algo::bundle_adjust::get_nested_algo_configuration(
      "sba_impl", config, d_->sba
      );
  maptk::algo::optimize_cameras::get_nested_algo_configuration(
      "camera_optimizer", config, d_->camera_optimizer
      );

  return config;
}


/// Set this algorithm's properties via a config block
void
hierarchical_bundle_adjust
::set_configuration(config_block_sptr config)
{
  d_->initial_sub_sample = config->get_value<unsigned int>("initial_sub_sample", d_->initial_sub_sample);
  d_->interpolation_rate = config->get_value<unsigned int>("interpolation_rate", d_->interpolation_rate);

  maptk::algo::bundle_adjust::set_nested_algo_configuration(
      "sba_impl", config, d_->sba
      );
  maptk::algo::optimize_cameras::set_nested_algo_configuration(
      "camera_optimizer", config, d_->camera_optimizer
      );
}


/// Check that the algorithm's configuration config_block is valid
bool
hierarchical_bundle_adjust
::check_configuration(config_block_sptr config) const
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

  if (!maptk::algo::bundle_adjust::check_nested_algo_configuration("sba_impl", config))
  {
    MAPTK_HSBA_CHECK_FAIL("sba_impl configuration invalid.");
  }
  if (!maptk::algo::optimize_cameras::check_nested_algo_configuration("camera_optimizer", config))
  {
    MAPTK_HSBA_CHECK_FAIL("camera_optimizer configuration invalid.");
  }

#undef MAPTK_HSBA_CHECK_FAIL

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
  unsigned int ir = d_->interpolation_rate;
  if (ir == 0)
  {
    ir = std::numeric_limits<unsigned int>::max();
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
        // concrete map of active cameras
        ac_map = active_cam_map->cameras();

      // pre-allocation of variables for performance
      size_t interval, jump,
             ir_l; // local interpolation rate as gaps available may be less than global rate
      double f;
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
            ir_l = min(ir, next_frm - cur_frm - 1);

            // Integral interval of each interpolation between cur and next frames
            // -> assuming even interpolation, this will divide evenly
            interval = (next_frm - cur_frm) / (ir_l + 1);

            for (jump = interval; cur_frm + jump < next_frm; jump += interval)
            {
              f = static_cast<double>(jump) / (next_frm - cur_frm);
              //cerr << "-> frm = " << (cur_frm + jump) << ", f = " << f << endl;
              interped_cams[cur_frm + jump] = interpolate_camera(cur_cam, next_cam, f);
            }
          }
        }
      }

      // Optimize new camers
      cerr << "optimizing new interpolated cameras (" << interped_cams.size() << " cams)" << endl;
      camera_map_sptr interped_cams_p(new simple_camera_map(interped_cams));
      { // scope block
        boost::timer::auto_cpu_timer t("\t- cameras optimization: %t sec CPU, %w sec wall\n");
        cerr << "\t- pre-optimization RMSE : " << reprojection_rmse(interped_cams_p->cameras(),
                                                                    landmarks->landmarks(),
                                                                    tracks->tracks()) << endl;
        d_->camera_optimizer->optimize(interped_cams_p, tracks, landmarks);
        cerr << "\t- post-optimization RMSE: " << reprojection_rmse(interped_cams_p->cameras(),
                                                                    landmarks->landmarks(),
                                                                    tracks->tracks()) << endl;
      }

      // adding optimized interpolated cameras to the map of existing cameras
      BOOST_FOREACH(camera_map::map_camera_t::value_type const& p, interped_cams_p->cameras())
      {
        ac_map[p.first] = p.second;
      }

      // Create a new active_cams_map to include interpolated cameras
      active_cam_map = camera_map_sptr(new simple_camera_map(ac_map));
      cerr << "Re-assiging new active cam map (" << active_cam_map->size() << " cams)" << endl
           << "\t- reprojection RMSE of combined map: "
           << reprojection_rmse(active_cam_map->cameras(),
                                landmarks->landmarks(),
                                tracks->tracks()) << endl;
    }
  } while (!done);

  // push up the resultant cameras
  cameras = active_cam_map;
}


}

}
