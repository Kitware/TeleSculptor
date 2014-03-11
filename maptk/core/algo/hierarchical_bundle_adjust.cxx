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

#include <maptk/core/algo/optimize_cameras.h>
#include <maptk/core/camera.h>
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
camera_map_sptr
subsample_cameras(camera_map_sptr cameras, unsigned n)
{
  // if sub-sample is 1, no sub-sampling occurs, just return a copy
  if (n == 1)
  {
    return camera_map_sptr(new simple_camera_map(cameras->cameras()));
  }

  camera_map::map_camera_t cams = cameras->cameras();
  camera_map::map_camera_t subsample;
  unsigned int i = 0;
  BOOST_FOREACH(camera_map::map_camera_t::value_type const& p, cams)
  {
    if (i % n == 0)
    {
      subsample[p.first] = p.second;
    }
    ++i;
  }
  return camera_map_sptr(new simple_camera_map(subsample));
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
                    "Number of cameras to fill in each iteration");

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

  // If interpolation rate is 0, then that means that all intermediate frames
  // should be interpolated on the first step. Due to how the algorithm
  // functions, set var to unsigned int max.
  if (d_->interpolation_rate == 0)
  {
    d_->interpolation_rate = std::numeric_limits<unsigned int>::max();
  }

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
 *  - cameras we are given are in sequence (no previous sub-sampling)
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
  //frame_id_t orig_max_frame = cameras->cameras().rbegin()->first;
  size_t num_orig_cams = cameras->size();

  // sub-sample cameras
  camera_map_sptr active_cam_map = subsample_cameras(cameras, d_->initial_sub_sample);

  // need to have at least 2 cameras
  if (active_cam_map->size() < 2)
  {
    throw invalid_value("Camera map given is of insufficient length.");
  }

  bool done = false;
  do
  {
    d_->sba->optimize(active_cam_map, landmarks, tracks);

    // If we've just completed SBA with all original frames in the new map,
    // then we're done.
    if (active_cam_map->size() == num_orig_cams)
    {
      done = true;
    }

    // perform interpolation between frames that have gaps in between them
    else
    {
      // Inserting interpolated cameras into separate map to be merged after
      // while loop
      camera_map::map_camera_t interped_cams,
                               c_map = active_cam_map->cameras();
      camera_map::map_camera_t::const_iterator it = c_map.begin();

      // pre-allocation of variables
      size_t jump;
      double f;
      frame_id_t cur_frm, next_frm;
      camera_sptr cur_cam, next_cam;

      // Iterate through frames and cameras, interpolating across gaps when found
      // ASSUMING even interpolation for now
      while (it != c_map.end())
      {
        cur_frm = it->first;
        cur_cam = it->second;
        ++it;

        // If we're not at the end of the active camera sequence
        if (it != c_map.end())
        {
          next_frm = it->first;
          next_cam = it->second;

          // assuming even interpolation, this will divide evenly
          jump = (next_frm - cur_frm) / (d_->interpolation_rate + 1);
          for (; cur_frm + jump < next_frm; jump += jump)
          {
            f = static_cast<double>(jump) / (next_frm - cur_frm);
            interped_cams[cur_frm + jump] = interpolate_camera(cur_cam, next_cam, f);
          }
        }
        // This should add every camera that was in the previous iteration,
        // makeing interped_cams the new master map after this loop.
        interped_cams[cur_frm] = cur_cam;
      }

      // Create a new active_cams_map to include interpolated cameras
      active_cam_map = camera_map_sptr(new simple_camera_map(interped_cams));

      // Optimize new camers
      // TODO: QUESTION: Should this be done on pre or post merged cameras???
      d_->camera_optimizer->optimize(active_cam_map, tracks, landmarks);
    }
  } while (!done);

  // push up the resultant cameras
  cameras = active_cam_map;
}


}

}
