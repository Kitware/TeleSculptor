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
#include <maptk/logging_macros.h>
#include <maptk/matrix.h>

#include <Eigen/LU>


#define LOGGING_PREFIX "compute_ref_homography_default"


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
    minimum_inliers( 4 ),
    frames_since_reset( 0 ),
    allow_ref_frame_regression( true ),
    min_ref_frame( 0 )
  {
  }

  priv( const priv& other )
  : use_backproject_error( other.use_backproject_error ),
    backproject_threshold_sqr( other.backproject_threshold_sqr ),
    forget_track_threshold( other.forget_track_threshold ),
    min_track_length( other.min_track_length ),
    inlier_scale( other.inlier_scale ),
    minimum_inliers( other.minimum_inliers ),
    frames_since_reset( other.frames_since_reset ),
    allow_ref_frame_regression( other.allow_ref_frame_regression ),
    min_ref_frame( other.min_ref_frame )
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

  /// Minimum points number of matching points between source and reference
  /// images when computing homography
  unsigned minimum_inliers;

  /// Buffer storing track extensions
  track_info_buffer_sptr buffer;

  /// Pointer to homography estimator
  algo::estimate_homography_sptr h_estimator;

  /// Number of frames since last new reference frame declared
  unsigned frames_since_reset;

  /// If we should allow reference frame regression or not when determining the
  /// earliest reference frame of active tracks.
  bool allow_ref_frame_regression;

  /// Minimum allowable reference frame. This is updated when homography
  /// estimation fails.
  frame_id_t min_ref_frame;


  /// Estimate the homography between two corresponding points sets
  /**
   * Check for homography validity.
   *
   * Output homography describes transformation from pts_src to pts_dst.
   *
   * If estimate homography is deemed bad, false is returned and the
   * homography passed to \p out_h is not modified.
   */
  bool compute_homography(std::vector<vector_2d> const &pts_src,
                          std::vector<vector_2d> const &pts_dst,
                          homography & out_h) const
  {
    bool is_bad_homog = false;
    homography tmp_h;

    // Make sure that we have at least the minimum number of points to match
    // between source and destination
    if ( pts_src.size() < this->minimum_inliers ||
         pts_dst.size() < this->minimum_inliers )
    {
      LOG_WARNING( LOGGING_PREFIX,
                   "Insufficient point pairs given to match. " <<
                   "Given " << pts_src.size() << " but require at least " << this->minimum_inliers );
      is_bad_homog = true;
    }
    else
    {
      std::vector<bool> inliers;
      tmp_h = this->h_estimator->estimate( pts_src, pts_dst, inliers, this->inlier_scale );

      // Check for positive inlier count
      unsigned inlier_count = 0;
      BOOST_FOREACH(bool b, inliers)
      {
        if ( b )
          ++inlier_count;
      }
      LOG_INFO( LOGGING_PREFIX,
                "Inliers after estimation: " << inlier_count );
      if ( inlier_count < this->minimum_inliers )
      {
        LOG_WARNING( LOGGING_PREFIX,
                     "Insufficient inliers after estimation. Require " << this->minimum_inliers );
        is_bad_homog = true;
      }
    }

    // Checking homography output for invertability and invalid values
    // Only need to try this if a supposed valid homog was estimated above
    if ( !is_bad_homog )
    {
      try
      {
        // would throw if not invertable
        homography inverse_test = maptk::inverse( tmp_h );

        // Checking for non-finite values in estimated and inversed homography
        unsigned i, j;
        for( i = 0; i < 3; ++i )
        {
          for ( j = 0; j < 3; ++j )
          {
            if ( !boost::math::isfinite( tmp_h(i,j) ) || !boost::math::isfinite( inverse_test(i,j) ) )
            {
              is_bad_homog = true;
              LOG_WARNING( LOGGING_PREFIX,
                           "Found non-finite values in estimated homography. Bad homography." );
              break;
            }
          }
          if ( is_bad_homog )
          {
            break;
          }
        }
      }
      catch (...)
      {
        LOG_WARNING( LOGGING_PREFIX,
                     "Homography non-invertable. Bad homography." );
        is_bad_homog = true;
      }
    }

    if ( !is_bad_homog )
    {
      out_h = tmp_h;
    }

    return is_bad_homog;
  }

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

  // parameterize number of matching points threshold (currently locked to >= 4)
  config->set_value("min_matches_threshold", d_->minimum_inliers,
                    "Minimum number of matches required between source and "
                    "reference planes for valid homography estimation.");
  config->set_value("allow_ref_frame_regression", d_->allow_ref_frame_regression,
                    "Allow for the possibility of a frame, N, to have a "
                    "reference frame, A, when a frame M < N has a reference "
                    "frame B > A (assuming frames were sequentially iterated "
                    "over with this algorithm).");

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
  d_->minimum_inliers = config->get_value<int>( "min_matches_threshold" );
  d_->allow_ref_frame_regression = config->get_value<bool>( "allow_ref_frame_regression" );

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
  LOG_DEBUG( LOGGING_PREFIX,
             "Starting ref homography estimation for frame " << frame_number );

  // Get active tracks for the current frame
  std::vector< track_sptr > active_tracks = tracks->active_tracks( frame_number )->tracks();

  // This is either the first frame, or a new reference frame
  if( !d_->buffer )
  {
    d_->buffer = track_info_buffer_sptr( new track_info_buffer_t() );
    d_->frames_since_reset = 0;
  }

  track_info_buffer_sptr new_buffer( new track_info_buffer_t() );
  std::vector< track_sptr > new_tracks;
  reset_active_flags( d_->buffer );

  // Flag tracks on this frame as new tracks, or "active" tracks, or tracks
  // that are not new.
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
  LOG_DEBUG( LOGGING_PREFIX,
             active_tracks.size() << " tracks on current frame (" <<
             (active_tracks.size() - new_tracks.size()) << " active, " <<
             new_tracks.size() << " new)" );

  // Add active tracks to new buffer, skipping those that we haven't seen in
  // a while.
  frame_id_t earliest_ref = std::numeric_limits<frame_id_t>::max();

  BOOST_FOREACH( track_info_t& ti, *(d_->buffer) )
  {
    if( ti.active || ++ti.missed_count < d_->forget_track_threshold )
    {
      new_buffer->push_back( ti );
    }

    // Save earliest reference frame of active tracks
    // If not allowing regression, take max against min_ref_frame
    if( ti.active && ti.ref_id < earliest_ref
        && (d_->allow_ref_frame_regression || (earliest_ref >= d_->min_ref_frame) ) )
    {
      earliest_ref = ti.ref_id;
    }
  }
  LOG_DEBUG( LOGGING_PREFIX,
             "Earliest Ref: " << earliest_ref );

  // Add new tracks to buffer.
  BOOST_FOREACH( track_sptr trk, new_tracks )
  {
    track::history_const_itr itr = trk->find( frame_number );

    if( itr != trk->end() && itr->feat )
    {
      track_info_t new_entry;

      new_entry.tid = trk->id();
      new_entry.ref_loc = vector_2d( itr->feat->loc() );
      new_entry.ref_id = frame_number;
      new_entry.active = false; // don't want to use this track on this frame
      new_entry.trk = trk;

      new_buffer->push_back( new_entry );
    }
  }

  // Ensure that the vector is still sorted. Chances are it still is and
  // this is a simple linear scan of the vector to ensure this.
  // This is needed for the find_track function's use of std::lower_bound
  // to work as expected.
  std::sort( d_->buffer->begin(), d_->buffer->end(), compare_ti );

  // Generate points to feed into homography regression
  std::vector<vector_2d> pts_ref, pts_cur;

  // Accept tracks that either stretch back to the reset point, or satisfy the
  // minimum track length parameter.
  size_t track_size_thresh = std::min( d_->min_track_length, d_->frames_since_reset + 1 );

  // Collect cur/ref points from track infos that have earliest-frame references
  BOOST_FOREACH( track_info_t& ti, *new_buffer )
  {
    if( ti.active && ti.is_good &&
        ti.ref_id == earliest_ref &&
        ti.trk->size() >= track_size_thresh )
    {
      track::history_const_itr itr = ti.trk->find( frame_number );

      if( itr->feat )
      {
        pts_ref.push_back( ti.ref_loc );
        pts_cur.push_back( itr->feat->loc() );
      }
    }
    // If the track is active and have a state on the earliest ref frame,
    // also include those points for homography estimation.
  }
  LOG_DEBUG( LOGGING_PREFIX,
             "Using " << pts_ref.size() << " points for estimation" );

  // Compute homography if possible
  homography h; // raw homography transform
  bool bad_homog = d_->compute_homography(pts_cur, pts_ref, h);

  // If the homography is bad, output an identity
  f2f_homography_sptr output;

  if( bad_homog )
  {
    LOG_DEBUG( LOGGING_PREFIX, "estimation FAILED" );
    // Start of new shot. Both frames the same and identity transform.
    output = f2f_homography_sptr( new f2f_homography( frame_number ) );
    d_->frames_since_reset = 0;
    d_->min_ref_frame = frame_number;
  }
  else
  {
    LOG_DEBUG( LOGGING_PREFIX, "estimation SUCCEEDED" );
    // extend current shot
    output = f2f_homography_sptr( new f2f_homography( h, frame_number, earliest_ref ) );
    output->normalize();
  }

  // Update track infos based on homography estimation result
  //  - With a valid homography, transform the reference location of active
  //    tracks with a different reference frame than the current earliest_ref
  BOOST_FOREACH( track_info_t& ti, *new_buffer )
  {
    track::history_const_itr itr = ti.trk->find( frame_number );

    // skip updating track items for tracks that don't have a state on this
    // frame, or a state without a feature (location)
    if ( itr == ti.trk->end() || !(itr->feat) )
    {
      continue
    }

    if ( !bad_homog )
    {
      // Update reference locations of active tracks that don't point to the
      // earliest_ref, and tracks that were just initialized (ref_id =
      // current_frame).
      if( (ti.active && ti.ref_id != earliest_ref) || ti.ref_id == frame_number )
      {
        ti.ref_loc = homography_map(*output, ti.ref_loc);
        ti.ref_id = output->to_id();
      }
      // Test back-projection on active tracks that we did not just set ref_loc
      // of.
      else if( d_->use_backproject_error && ti.active )
      {
        vector_2d warped = homography_map(*output, itr->feat->loc());
        double dist_sqr = ( warped - ti.ref_loc ).squaredNorm();

        if( dist_sqr > d_->backproject_threshold_sqr )
        {
          ti.is_good = false;
        }
      }
    }
    // If not allowing ref regression, update reference loc and id of
    // active tracks to the current frame on estimation failure.
    else if ( !d_->allow_ref_frame_regression && ti.active )
    {
      LOG_DEBUG( LOGGING_PREFIX,
                 "Resetting tID(" << ti.tid << ") " <<
                 "reference to frame " << frame_number );
      ti.ref_loc = vector_2d( itr->feat->loc() );
      ti.ref_id = frame_number;
    }
  }

  // Increment counter, update buffers
  d_->frames_since_reset++;
  d_->buffer = new_buffer;

  return output;
}

} // end namespace core

} // end namespace maptk
