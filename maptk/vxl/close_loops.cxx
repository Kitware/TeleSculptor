/*ckwg +5
 * Copyright 2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

/**
 * \file
 * \brief Implementation of \link maptk::algo::close_loops_bad_frames_only
 *        close_loops_bad_frames_only \endlink
 */

#include <maptk/vxl/close_loops.h>
#include <maptk/core/algo/estimate_homography.h>

#include <algorithm>
#include <string>
#include <fstream>

#include <boost/bind.hpp>
#include <boost/foreach.hpp>
#include <boost/circular_buffer.hpp>


namespace maptk
{

namespace vxl
{


// Data stored for every detected checkpoint
typedef std::pair< frame_id_t, homography_collection_sptr > checkpoint_entry_t;


// Buffer type for detected checkpoints
typedef boost::circular_buffer< checkpoint_entry_t > checkpoint_buffer_t;


// Data stored for every active track
struct extra_track_info
{
  // Should this point be used in homography regression?
  bool is_good;

  // Pointer to the track object this class extends
  track_sptr trk;
};


// Private implementation class
class close_loops::priv
{
public:

  priv()
  : long_term_closure_enabled_( true ),
    max_search_frames_( 10000 ),
    checkpoint_percent_overlap_( 0.40 ),
    homography_filename_( "" )
  {
  }

  priv( const priv& other )
  : long_term_closure_enabled_( other.long_term_closure_enabled_ ),
    max_search_frames_( other.max_search_frames_ ),
    checkpoint_percent_overlap_( other.checkpoint_percent_overlap_ ),
    homography_filename_( other.homography_filename_ )
  {
  }

  ~priv()
  {
  }

  /// Is long term loop closure enabled?
  bool long_term_closure_enabled_;

  /// Maximum past search distance in terms of number of frames.
  unsigned max_search_frames_;

  /// Term which controls when we make new loop closure checkpoints.
  double checkpoint_percent_overlap_;

  /// Output filename for homographies
  std::string homography_filename_;

  /// Buffer storing past homographies for checkpoint frames
  checkpoint_buffer_t buffer_;

  /// Previous frame active track info
  std::vector< extra_track_info > track_info_;
};


close_loops
::close_loops()
: d_( new priv() )
{
}


close_loops
::close_loops( const close_loops& other )
: close_loops_bad_frames_only( other ),
  d_( new priv( *other.d_ ) )
{
}


close_loops
::~close_loops()
{
}


config_block_sptr
close_loops
::get_configuration() const
{
  // get base config from base class
  config_block_sptr config = close_loops_bad_frames_only::get_configuration();

  // Sub-algorithm implementation name + sub_config block
  // - Homography estimator algorithm
  maptk::algo::estimate_homography::get_nested_algo_configuration
    ( "homography_estimator", config, h_estimator_ );

  // Loop closure parameters
  config->set_value("long_term_closure_enabled", d_->long_term_closure_enabled_,
                    "Is long term loop closure enabled?");
  config->set_value("max_search_frames", d_->max_search_frames_,
                    "Maximum past search distance in terms of number of frames.");
  config->set_value("checkpoint_percent_overlap", d_->checkpoint_percent_overlap_,
                    "Term which controls when we make new loop closure checkpoints.");
  config->set_value("homography_filename", d_->homography_filename_,
                    "Optional output location for a homography text file.");

  return config;
}


void
close_loops
::set_configuration( config_block_sptr in_config )
{
  // Starting with our generated config_block to ensure that assumed values are present
  // An alternative is to check for key presence before performing a get_value() call.
  config_block_sptr config = this->get_configuration();
  config->merge_config( in_config );

  // Setting nested algorithm instances via setter methods instead of directly
  // assigning to instance property.
  maptk::algo::estimate_homography_sptr hest;
  maptk::algo::estimate_homography::set_nested_algo_configuration
    ( "homography_estimator", config, hest );
  h_estimator_ = hest;

  // Settings for bad frame detection
  d_->long_term_closure_enabled_ = config->get_value<bool>( "long_term_closure_enabled" );
  d_->max_search_frames_ = config->get_value<unsigned>( "max_search_frames" );
  d_->checkpoint_percent_overlap_ = config->get_value<double>( "checkpoint_percent_overlap" );
  d_->homography_filename_ = config->get_value<double>( "homography_filename" );

  // Set buffer capacity
  d_->buffer_.set_capacity( d_->max_search_frames_ );
}


bool
close_loops
::check_configuration( config_block_sptr config ) const
{
  return close_loops_bad_frames_only::check_configuration( config );
}


// Compute overlap between the two frames of the specified size
double
compute_percent_overlap( const f2f_homography& homog,
                         const unsigned image_cols,
                         const unsigned image_rows )
{
  return 0.0;
}


// Compute new homographies for the current frame
homography_collection_sptr
compute_new_homographies( frame_id_t frame_number,
                          track_set_sptr input )
{
  return homography_collection_sptr();
}


// Perform stitch operation
track_set_sptr
close_loops
::stitch( frame_id_t frame_number,
          image_container_sptr image,
          track_set_sptr input ) const
{
  // Perform bad frame detection for this frame (if enabled)
  track_set_sptr updated_set =
    close_loops_bad_frames_only::stitch( frame_number, image, input );

  // Compute new homographies for this frame
  homography_collection_sptr new_homographies =
    compute_new_homographies( frame_number, updated_set );

  // Write out homographies if enabled
  if( !d_->homography_filename_.empty() )
  {
    std::ofstream fout( d_->homography_filename_.c_str(), std::ios::app );
    fout << *(new_homographies->current_to_world()) << std::endl << std::endl;
    fout.close();
  }

  // Determine if this is a new checkpoint frame
  // []

  // If this is a new checkpoint
  if( 0 )
  {
    // Add homography info to buffer
    // []
  }

  // Perform matching to any past checkpoints we want to test
  // []

  // Return updated track set
  return updated_set;
}


} // end namespace vxl

} // end namespace maptk
