/*ckwg +29
 * Copyright 2014 by Kitware, Inc.
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
 * \brief Implementation of \link maptk::algo::compute_ref_homography_core
 *        compute_ref_homography_core \endlink
 */

#include "compute_ref_homography_core.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <set>
#include <vector>

#include <boost/foreach.hpp>
#include <boost/math/special_functions/fpclassify.hpp>

#include <maptk/algo/estimate_homography.h>
#include <Eigen/LU>


namespace maptk
{

namespace core
{

namespace
{


// Extra data stored for every active track
struct track_info_t
{
  // Track ID for the given track this struct extends
  track_id_t tid;

  // Location of this track in the reference frame
  vector_2d ref_loc;

  // Is the ref loc valid?
  bool ref_loc_valid;

  // Reference frame ID
  frame_id_t ref_id;

  // Does this point satisfy all required backprojection properties?
  bool is_good;

  // The number of times we haven't seen this track in the active set
  unsigned missed_count;

  // On the current frame was this track updated?
  bool active;

  // Pointer to the latest instance of the track containing the above id
  track_sptr trk;

  // Constructor.
  track_info_t()
  : ref_loc( 0.0, 0.0 ),
    ref_loc_valid( false ),
    is_good( true ),
    missed_count( 0 ),
    active( false )
  {}
};


// Buffer type for storing the extra track info for all tracks
typedef std::vector< track_info_t > track_info_buffer_t;

// Pointer to a track info buffer
typedef boost::shared_ptr< track_info_buffer_t > track_info_buffer_sptr;


// Helper function for sorting tis
bool
compare_ti( const track_info_t& c1, const track_info_t& c2 )
{
  return ( c1.tid < c2.tid );
}


// Find a track in a given buffer
track_info_buffer_t::iterator
find_track( const track_sptr& trk, track_info_buffer_sptr& buffer )
{
  track_info_t ti;
  ti.tid = trk->id();
  return std::lower_bound( buffer->begin(), buffer->end(), ti, compare_ti );
}


// Reset all is found flags
void
reset_active_flags( track_info_buffer_sptr buffer )
{
  BOOST_FOREACH( track_info_t& ti, *buffer )
  {
    ti.active = false;
  }
}


} // end namespace anonymous


// Private implementation class
class compute_ref_homography_core::priv
{
public:

  priv()
  : use_backproject_error( false ),
    backproject_threshold_sqr( 16.0 ),
    forget_track_threshold( 5 ),
    min_track_length( 1 ),
    inlier_scale( 2.0 ),
    frames_since_reset( 0 ),
    last_homog( 0 )
  {
  }

  priv( const priv& other )
  : use_backproject_error( other.use_backproject_error ),
    backproject_threshold_sqr( other.backproject_threshold_sqr ),
    forget_track_threshold( other.forget_track_threshold ),
    min_track_length( other.min_track_length ),
    inlier_scale( other.inlier_scale ),
    frames_since_reset( other.frames_since_reset ),
    last_homog( other.last_homog )
  {
  }

  ~priv()
  {
  }

  /// Should we remove extra points if the backproject error is high?
  bool use_backproject_error;

  /// Backprojection threshold in terms of L2 distance (number of pixels)
  double backproject_threshold_sqr;

  /// After how many frames should we forget all info about a track?
  unsigned forget_track_threshold;

  /// Minimum track length to use for homography regression
  unsigned min_track_length;

  /// The scale of inlier points used for homography calculation
  double inlier_scale;

  /// Buffer storing track extensions
  track_info_buffer_sptr buffer;

  /// Pointer to homography estimator
  algo::estimate_homography_sptr h_estimator;

  /// Number of frames since last new reference frame declared
  unsigned frames_since_reset;

  /// Last known transformation
  f2f_homography last_homog;

};


compute_ref_homography_core
::compute_ref_homography_core()
: d_( new priv() )
{
}


compute_ref_homography_core
::compute_ref_homography_core( const compute_ref_homography_core& other )
: d_( new priv( *other.d_ ) )
{
}


compute_ref_homography_core
::~compute_ref_homography_core()
{
}


std::string
compute_ref_homography_core
::description() const
{
  return "Default online sequential-frame reference homography estimator";
}


config_block_sptr
compute_ref_homography_core
::get_configuration() const
{
  // get base config from base class
  config_block_sptr config = algorithm::get_configuration();

  // Sub-algorithm implementation name + sub_config block
  // - Homography estimator algorithm
  algo::estimate_homography::get_nested_algo_configuration( "estimator", config, d_->h_estimator );

  // Other parameters
  config->set_value("use_backproject_error", d_->use_backproject_error,
                    "Should we remove extra points if the backproject error is high?");
  config->set_value("backproject_threshold", std::sqrt( d_->backproject_threshold_sqr ),
                    "Backprojection threshold in terms of L2 distance (number of pixels)");
  config->set_value("forget_track_threshold", d_->forget_track_threshold,
                    "After how many frames should we forget all info about a track?");
  config->set_value("min_track_length", d_->min_track_length,
                    "Minimum track length to use for homography regression");
  config->set_value("inlier_scale", d_->inlier_scale,
                    "The acceptable error distance (in pixels) between warped "
                    "and measured points to be considered an inlier match.");

  return config;
}


void
compute_ref_homography_core
::set_configuration( config_block_sptr in_config )
{
  // Starting with our generated config_block to ensure that assumed values are present
  // An alternative is to check for key presence before performing a get_value() call.
  config_block_sptr config = this->get_configuration();
  config->merge_config( in_config );

  // Setting nested algorithm instances via setter methods instead of directly
  // assigning to instance property.
  algo::estimate_homography::set_nested_algo_configuration( "estimator", config, d_->h_estimator );

  // Read other parameters
  d_->use_backproject_error = config->get_value<bool>( "use_backproject_error" );
  d_->backproject_threshold_sqr = config->get_value<double>( "backproject_threshold" );
  d_->forget_track_threshold = config->get_value<unsigned>( "forget_track_threshold" );
  d_->min_track_length = config->get_value<unsigned>( "min_track_length" );
  d_->inlier_scale = config->get_value<double>( "inlier_scale" );

  // Square the threshold ahead of time for efficiency
  d_->backproject_threshold_sqr = d_->backproject_threshold_sqr *
                                  d_->backproject_threshold_sqr;
}


bool
compute_ref_homography_core
::check_configuration(config_block_sptr config) const
{
  return
  (
    algo::estimate_homography::check_nested_algo_configuration( "estimator", config )
  );
}


// Perform actual current to reference frame estimation
f2f_homography_sptr
compute_ref_homography_core
::estimate( frame_id_t frame_number,
            track_set_sptr tracks ) const
{
  // Get active tracks for the current frame
  std::vector< track_sptr > active_tracks = tracks->active_tracks( frame_number )->tracks();

  // This is either the first frame, or a new reference frame
  if( !d_->buffer )
  {
    d_->buffer = track_info_buffer_sptr( new track_info_buffer_t() );
    d_->last_homog = f2f_homography( frame_number );
    d_->frames_since_reset = 0;
  }

  track_info_buffer_sptr new_buffer( new track_info_buffer_t() );
  std::vector< track_sptr > new_tracks;
  reset_active_flags( d_->buffer );

  // Determine new and active tracks
  BOOST_FOREACH( track_sptr trk, active_tracks )
  {
    track_info_buffer_t::iterator p = find_track( trk, d_->buffer );

    // The track was active
    if( p != d_->buffer->end() )
    {
      p->active = true;
      p->missed_count = 0;
      p->trk = trk;
    }
    else
    {
      new_tracks.push_back( trk );
    }
  }

  // Add active tracks to buffer, skipping any terminated ones.
  frame_id_t earliest_ref = std::numeric_limits<frame_id_t>::max();

  BOOST_FOREACH( track_info_t& ti, *(d_->buffer) )
  {
    if( ti.active || ++ti.missed_count < d_->forget_track_threshold )
    {
      new_buffer->push_back( ti );
    }
    if( ti.active && ti.ref_id < earliest_ref )
    {
      earliest_ref = ti.ref_id;
    }
  }

  // Add new tracks to buffer.
  BOOST_FOREACH( track_sptr trk, new_tracks )
  {
    track::history_const_itr itr = trk->find( frame_number );

    if( itr != trk->end() && itr->feat )
    {
      track_info_t new_entry;

      new_entry.tid = trk->id();
      new_entry.ref_loc = vector_2d( itr->feat->loc() );
      new_entry.active = false; // don't want to use this track on this frame
      new_entry.trk = trk;

      new_buffer->push_back( new_entry );
    }
  }

  // Ensure that the vector is still sorted. Chances are it still is and
  // this is a simple linear scan of the vector to ensure this.
  std::sort( d_->buffer->begin(), d_->buffer->end(), compare_ti );

  // Generate points to feed into homography regression
  std::vector<vector_2d> pts_ref, pts_cur;

  size_t track_size_thresh = std::min( d_->min_track_length, d_->frames_since_reset + 1 );

  BOOST_FOREACH( track_info_t& ti, *new_buffer )
  {
    if( ti.active && ti.is_good &&
        ti.ref_loc_valid && ti.ref_id == earliest_ref &&
        ti.trk->size() >= track_size_thresh )
    {
      track::history_const_itr itr = ti.trk->find( frame_number );

      if( itr->feat )
      {
        pts_ref.push_back( ti.ref_loc );
        pts_cur.push_back( itr->feat->loc() );
      }
    }
  }

  // Compute homography if possible
  bool bad_homog = false;

  homography h; // raw homography transform

  if( pts_ref.size() > 3 && pts_cur.size() > 3 )
  {
    std::vector<bool> inliers;
    h = d_->h_estimator->estimate( pts_cur, pts_ref, inliers, d_->inlier_scale );
  }
  else
  {
    bad_homog = true;
  }

  // Check homography output
  try
  {
    using namespace std;

    // Invertible test
    homography inverse;
    bool inv_valid;
    h.computeInverseWithCheck(inverse, inv_valid);
    bad_homog = !inv_valid || bad_homog;

    // Check for invalid values
    for( unsigned i = 0; i < 3; i++ )
    {
      for( unsigned j = 0; j < 3; j++ )
      {
        if( !boost::math::isfinite( h(i,j) ) || !boost::math::isfinite( inverse(i,j) ) )
        {
          bad_homog = true;
          break;
        }
      }
    }
  }
  catch(...)
  {
    bad_homog = true;
  }

  // If the homography is bad, output an identity
  f2f_homography_sptr output;

  if( bad_homog )
  {
    // Start of new shot. Both frames the same and identity transform.
    output = f2f_homography_sptr( new f2f_homography( frame_number ) );
    d_->frames_since_reset = 0;
  }
  else
  {
    // extend current shot
    output = f2f_homography_sptr( new f2f_homography( h, frame_number, earliest_ref ) );
    output->normalize();
  }

  vector_3d tmp_3d;
  BOOST_FOREACH( track_info_t& ti, *new_buffer )
  {
    // Update reference locations for existing tracks using new homography
    if( !ti.ref_loc_valid )
    {
      ti.ref_loc = homography_map(*output, ti.ref_loc);
      ti.ref_loc_valid = true;
      ti.ref_id = output->to_id();
    }
    // Set is good flag
    else if( ti.active && !bad_homog && d_->use_backproject_error )
    {
      track::history_const_itr itr = ti.trk->find( frame_number );

      if( itr != ti.trk->end() && itr->feat )
      {
        vector_2d warped = homography_map(*output, itr->feat->loc());
        double dist_sqr = ( warped - ti.ref_loc ).squaredNorm();

        if( dist_sqr > d_->backproject_threshold_sqr )
        {
          ti.is_good = false;
        }
      }
    }
  }

  // Increment counter, update buffers
  d_->frames_since_reset++;
  d_->buffer = new_buffer;
  d_->last_homog = *output;

  return output;
}

} // end namespace core

} // end namespace maptk
