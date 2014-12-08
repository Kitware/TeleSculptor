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
 * \brief Implementation of \link maptk::vxl::close_loops_homography_guided
 *        close_loops \endlink
 */

#include "close_loops_homography_guided.h"

#include <algorithm>
#include <fstream>
#include <iostream>
#include <string>

#include <boost/bind.hpp>
#include <boost/circular_buffer.hpp>
#include <boost/foreach.hpp>

#include <maptk/algo/compute_ref_homography.h>
#include <maptk/algo/match_features.h>
#include <maptk/plugins/vxl/compute_homography_overlap.h>


namespace maptk
{

namespace vxl
{

namespace
{


// Data stored for every detected checkpoint
class checkpoint_entry_t
{
public:

  // Constructor
  checkpoint_entry_t()
  : fid( 0 ),
    src_to_ref( new f2f_homography( fid ) )
  {
  }

  // Constructor
  checkpoint_entry_t( frame_id_t id, f2f_homography_sptr h )
  : fid( id ),
    src_to_ref( h )
  {
  }

  // Frame ID of checkpoint
  frame_id_t fid;

  // Source to ref homography
  f2f_homography_sptr src_to_ref;
};


// Buffer type for detected checkpoints
typedef boost::circular_buffer< checkpoint_entry_t > checkpoint_buffer_t;

// Buffer reverse iterator
typedef checkpoint_buffer_t::reverse_iterator buffer_ritr;


// Functor to help remove tracks from vector
bool
track_id_in_set( track_sptr trk_ptr, std::set<track_id_t>* set_ptr )
{
  return set_ptr->find( trk_ptr->id() ) != set_ptr->end();
}


// If possible convert a src1 to ref and src2 to ref homography to a src2 to src1 homography
bool
convert( const f2f_homography_sptr& src1_to_ref,
         const f2f_homography_sptr& src2_to_ref,
         f2f_homography& src2_to_src1 )
{
  try
  {
    src2_to_src1 = src1_to_ref->inverse() * (*src2_to_ref);
    return true;
  }
  catch(...)
  {
    std::cerr << "Warn: Invalid homography received" << std::endl;
  }

  src2_to_src1 = *src2_to_ref;
  return false;
}


} // end namespace anonymous


/// Private implementation class
class close_loops_homography_guided::priv
{
public:

  priv()
  : enabled_( true ),
    max_checkpoint_frames_( 10000 ),
    checkpoint_percent_overlap_( 0.70 ),
    homography_filename_( "" )
  {
  }

  priv( const priv& other )
  : enabled_( other.enabled_ ),
    max_checkpoint_frames_( other.max_checkpoint_frames_ ),
    checkpoint_percent_overlap_( other.checkpoint_percent_overlap_ ),
    homography_filename_( other.homography_filename_ )
  {
  }

  ~priv()
  {
  }

  /// Is long term loop closure enabled?
  bool enabled_;

  /// Maximum past search distance in terms of number of checkpoints.
  unsigned max_checkpoint_frames_;

  /// Term which controls when we make new loop closure checkpoints.
  double checkpoint_percent_overlap_;

  /// Output filename for homographies
  std::string homography_filename_;

  /// Buffer storing past homographies for checkpoint frames
  checkpoint_buffer_t buffer_;

  /// Reference frame homography computer
  maptk::algo::compute_ref_homography_sptr ref_computer_;

  /// The feature matching algorithm to use
  maptk::algo::match_features_sptr matcher_;
};


close_loops_homography_guided
::close_loops_homography_guided()
: d_( new priv() )
{
}


close_loops_homography_guided
::close_loops_homography_guided( const close_loops_homography_guided& other )
: d_( new priv( *other.d_ ) )
{
}


close_loops_homography_guided
::~close_loops_homography_guided()
{
}


config_block_sptr
close_loops_homography_guided
::get_configuration() const
{
  // get base config from base class
  config_block_sptr config = algorithm::get_configuration();

  // Sub-algorithm implementation name + sub_config block
  // - Homography estimator algorithm
  maptk::algo::compute_ref_homography::get_nested_algo_configuration( "ref_computer", config, d_->ref_computer_ );

  // - Feature Matcher algorithm
  maptk::algo::match_features::get_nested_algo_configuration( "feature_matcher", config, d_->matcher_ );

  // Loop closure parameters
  config->set_value("enabled", d_->enabled_,
                    "Is long term loop closure enabled?");
  config->set_value("max_checkpoint_frames", d_->max_checkpoint_frames_,
                    "Maximum past search distance in terms of number of checkpoints.");
  config->set_value("checkpoint_percent_overlap", d_->checkpoint_percent_overlap_,
                    "Term which controls when we make new loop closure checkpoints. "
                    "Everytime the percentage of tracked features drops below this "
                    "threshold, we generate a new checkpoint.");
  config->set_value("homography_filename", d_->homography_filename_,
                    "Optional output location for a homography text file.");

  return config;
}


void
close_loops_homography_guided
::set_configuration( config_block_sptr in_config )
{
  // Starting with our generated config_block to ensure that assumed values are present
  // An alternative is to check for key presence before performing a get_value() call.
  config_block_sptr config = this->get_configuration();
  config->merge_config( in_config );

  // Setting nested algorithm instances via setter methods instead of directly
  // assigning to instance property.
  maptk::algo::compute_ref_homography_sptr rc;
  maptk::algo::compute_ref_homography::set_nested_algo_configuration( "ref_computer", config, rc );
  d_->ref_computer_ = rc;

  maptk::algo::match_features_sptr mf;
  maptk::algo::match_features::set_nested_algo_configuration( "feature_matcher", config, mf );
  d_->matcher_ = mf;

  // Settings for bad frame detection
  d_->enabled_ = config->get_value<bool>( "enabled" );
  d_->max_checkpoint_frames_ = config->get_value<unsigned>( "max_checkpoint_frames" );
  d_->checkpoint_percent_overlap_ = config->get_value<double>( "checkpoint_percent_overlap" );
  d_->homography_filename_ = config->get_value<std::string>( "homography_filename" );

  // Set buffer capacity
  d_->buffer_.set_capacity( d_->max_checkpoint_frames_ );

  // Touch and reset output file
  if( !d_->homography_filename_.empty() )
  {
    std::ofstream fout( d_->homography_filename_.c_str() );
    fout.close();
  }
}


bool
close_loops_homography_guided
::check_configuration( config_block_sptr config ) const
{
  return
  (
    maptk::algo::compute_ref_homography::check_nested_algo_configuration( "ref_computer", config )
    &&
    maptk::algo::match_features::check_nested_algo_configuration( "feature_matcher", config )
  );
}


// Perform stitch operation
track_set_sptr
close_loops_homography_guided
::stitch( frame_id_t frame_number,
          image_container_sptr image,
          track_set_sptr input ) const
{
  if( !d_->enabled_ )
  {
    return input;
  }

  const unsigned int width = static_cast<unsigned int>(image->width());
  const unsigned int height = static_cast<unsigned int>(image->height());

  // Compute new homographies for this frame (current_to_ref)
  f2f_homography_sptr homog = d_->ref_computer_->estimate( frame_number, input );

  // Write out homographies if enabled
  if( !d_->homography_filename_.empty() )
  {
    std::ofstream fout( d_->homography_filename_.c_str(), std::ios::app );
    fout << *homog << std::endl;
    fout.close();
  }

  // Determine if this is a new checkpoint frame. Either the buffer is empty
  // and this is a new frame, this is a homography for a new reference frame,
  // or the overlap with the last checkpoint was less than a specified amount.
  f2f_homography tmp( frame_number );

  if( d_->buffer_.empty() ||
      !convert( d_->buffer_.back().src_to_ref, homog, tmp ) ||
      overlap( vnl_double_3x3( tmp.data() ), width, height ) < d_->checkpoint_percent_overlap_ )
  {
    d_->buffer_.push_back( checkpoint_entry_t( frame_number, homog ) );
  }

  // Perform matching to any past checkpoints we want to test
  enum { initial, non_intersection, reintersection } scan_state;

  buffer_ritr best_frame_to_test = d_->buffer_.rend();
  double best_frame_intersection = 0.0;
  scan_state = initial;

  for( buffer_ritr itr = d_->buffer_.rbegin(); itr != d_->buffer_.rend(); itr++ )
  {
    bool transform_valid = convert( itr->src_to_ref, homog, tmp );
    double po = 0.0;

    if( transform_valid )
    {
      po = overlap( vnl_double_3x3( tmp.data() ), width, height );
    }

    if( scan_state == reintersection )
    {
      if( !transform_valid || !po )
      {
        break;
      }

      if( po > best_frame_intersection )
      {
        best_frame_to_test = itr;
        best_frame_intersection = po;
      }
    }
    else if( scan_state == initial )
    {
      if( !transform_valid || !po )
      {
        scan_state = non_intersection;
      }
    }
    else if( transform_valid && po > 0 )
    {
      best_frame_to_test = itr;
      best_frame_intersection = po;
      scan_state = reintersection;
    }
  }

  // Perform matching operation to target frame if possible
  if( best_frame_to_test != d_->buffer_.rend() )
  {
    const frame_id_t prior_frame = best_frame_to_test->fid;

    // Get all tracks on target frame
    track_set_sptr prior_set = input->active_tracks( prior_frame );

    // Get all tracks on the current frame
    track_set_sptr current_set = input->active_tracks( frame_number );

    // Perform matching operation
    match_set_sptr mset = d_->matcher_->match(
      current_set->frame_features( frame_number ),
      current_set->frame_descriptors( frame_number ),
      prior_set->frame_features( prior_frame ),
      prior_set->frame_descriptors( prior_frame ) );

    // Test matcher results
    if( mset->size() > 0 ) // If matches are good
    {
      // Logging
      std::cout << "Stitching frames " << prior_frame << " and " << frame_number << std::endl;

      // Get all tracks, we will modify this
      std::vector< track_sptr > all_tracks = input->tracks();

      // Get all tracks on the past frame
      std::vector< track_sptr > prior_trks = prior_set->tracks();

      // Get all tracks on the current frame
      std::vector< track_sptr > current_trks = current_set->tracks();

      // Get all matches
      std::vector<match> matches = mset->matches();
      std::set<track_id_t> to_remove;

      for( unsigned i = 0; i < matches.size(); i++ )
      {
        if( prior_trks[ matches[i].second ]->append( *current_trks[ matches[i].first ] ) )
        {
          to_remove.insert( current_trks[ matches[i].first ]->id() );
        }
      }

      if( !to_remove.empty() )
      {
        all_tracks.erase(
          std::remove_if( all_tracks.begin(), all_tracks.end(), boost::bind( track_id_in_set, _1, &to_remove ) ),
          all_tracks.end()
        );
      }

      // Return updated set
      return track_set_sptr( new simple_track_set( all_tracks ) );
    }
  }

  // Return input set
  return input;
}


} // end namespace vxl

} // end namespace maptk
