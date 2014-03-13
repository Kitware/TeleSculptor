/*ckwg +5
 * Copyright 2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

/**
 * \file
 * \brief Implementation of \link maptk::algo::compute_ref_homography_default
 *        compute_ref_homography_default \endlink
 */

#include "compute_ref_homography_default.h"

#include <maptk/core/algo/estimate_homography.h>

#include <algorithm>
#include <iostream>
#include <set>
#include <vector>

#include <boost/foreach.hpp>


namespace maptk
{

namespace algo
{


// Extra data stored for every active track
struct extra_track_info
{
  // Track ID for the given track this struct extends
  track_id_t tid;

  // Location of this track in the reference frame
  homography_point ref_loc;

  // Is the ref loc valid?
  bool ref_loc_valid;

  // Should this point be used in homography regression?
  bool is_good;

  // The number of times we haven't seen this track as active
  unsigned missed_count;

  // Constructor.
  extra_track_info()
  : ref_loc( 0.0, 0.0 ),
    ref_loc_valid( false ),
    is_good( false ),
    missed_count( 0 )
  {}
};


// Buffer type for the extra track info
typedef std::vector< extra_track_info > track_ext_buffer_t;

// Pointer to a track info buffer
typedef boost::shared_ptr< track_ext_buffer_t > track_ext_buffer_sptr;

// Internal homography estimator type
typedef maptk::algo::estimate_homography_sptr estimator_sptr;


// Private implementation class
class compute_ref_homography_default::priv
{
public:

  priv()
  : use_backproject_error( false ),
    backproject_threshold_sqr( 16.0 ),
    forget_track_threshold( 10 ),
    min_track_length( 1 )
  {
  }

  priv( const priv& other )
  : use_backproject_error( other.use_backproject_error ),
    backproject_threshold_sqr( other.backproject_threshold_sqr ),
    forget_track_threshold( other.forget_track_threshold ),
    min_track_length( other.min_track_length )
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

  /// Buffer storing track extensions
  track_ext_buffer_sptr buffer;

  /// Pointer to homography estimator
  estimator_sptr h_estimator;

  /// Last known transformation
  f2f_homography_sptr last_homog;
};


compute_ref_homography_default
::compute_ref_homography_default()
: d_( new priv() )
{
}


compute_ref_homography_default
::compute_ref_homography_default( const compute_ref_homography_default& other )
: d_( new priv( *other.d_ ) )
{
}


compute_ref_homography_default
::~compute_ref_homography_default()
{
}


config_block_sptr
compute_ref_homography_default
::get_configuration() const
{
  // get base config from base class
  config_block_sptr config = algorithm::get_configuration();

  // Sub-algorithm implementation name + sub_config block
  // - Homography estimator algorithm
  estimate_homography::get_nested_algo_configuration( "estimator", config, d_->h_estimator );

  // Other parameters
  config->set_value("use_backproject_error", d_->use_backproject_error,
                    "Should we remove extra points if the backproject error is high?");
  config->set_value("backproject_threshold", d_->backproject_threshold_sqr,
                    "Backprojection threshold in terms of L2 distance (number of pixels)");
  config->set_value("forget_track_threshold", d_->forget_track_threshold,
                    "After how many frames should we forget all info about a track?");
  config->set_value("min_track_length", d_->min_track_length,
                    "Minimum track length to use for homography regression");

  return config;
}


void
compute_ref_homography_default
::set_configuration( config_block_sptr in_config )
{
  // Starting with our generated config_block to ensure that assumed values are present
  // An alternative is to check for key presence before performing a get_value() call.
  config_block_sptr config = this->get_configuration();
  config->merge_config( in_config );

  // Setting nested algorithm instances via setter methods instead of directly
  // assigning to instance property.
  estimate_homography::set_nested_algo_configuration( "estimator", config, d_->h_estimator );

  // Read other parameters
  d_->use_backproject_error = config->get_value<bool>("use_backproject_error");
  d_->backproject_threshold_sqr = config->get_value<double>("backproject_threshold");
  d_->forget_track_threshold = config->get_value<unsigned>("forget_track_threshold");
  d_->min_track_length = config->get_value<unsigned>("min_track_length");

  // Square the threshold ahead of time for efficiency
  d_->backproject_threshold_sqr = d_->backproject_threshold_sqr *
                                  d_->backproject_threshold_sqr;
}


bool
compute_ref_homography_default
::check_configuration(config_block_sptr config) const
{
  return
  (
    estimate_homography::check_nested_algo_configuration( "estimator", config )
  );
}


// Helper function for sorting etis
bool
compare_eti( const extra_track_info& c1, const extra_track_info& c2 )
{
  return c1.tid < c2.tid;
}


// Find a track in a given buffer
track_ext_buffer_t::iterator
find_track( const track_sptr& trk, track_ext_buffer_sptr& buffer )
{
  extra_track_info eti;
  eti.tid = trk->id();
  return std::lower_bound( buffer->begin(), buffer->end(), eti, compare_eti );
}

f2f_homography_sptr
compute_ref_homography_default
::estimate( frame_id_t frame_number,
            track_set_sptr tracks ) const
{
  // Get active tracks for the current frame
  std::vector< track_sptr > active_tracks = tracks->active_tracks( frame_number )->tracks();

  // This either is the first frame, or a new reference frame
  if( !d_->buffer )
  {
    d_->buffer = track_ext_buffer_sptr( new track_ext_buffer_t() );

    homography identity;
    identity.set_identity();

    d_->last_homog = f2f_homography_sptr( new f2f_homography( identity, frame_number, frame_number ) );
  }

  // Process new tracks, add to list, and remove old tracks.
  std::vector< track_sptr > new_tracks;

  BOOST_FOREACH( track_sptr trk, active_tracks )
  {
    track_ext_buffer_t::iterator p = find_track( trk, d_->buffer );

    if( p != d_->buffer->end() )
    {

    }
    else
    {

    }
  }

  // Ensure that the vector is still sorted. Chances are it still is and
  // this is a simple linear scan of the vector to ensure this.
  std::sort( d_->buffer->begin(), d_->buffer->end(), compare_eti );

  // Compute homography if possible
  // []

  // Update reference locations for existing tracks using homography
  // []

  return f2f_homography_sptr();
}

} // end namespace algo

} // end namespace maptk
