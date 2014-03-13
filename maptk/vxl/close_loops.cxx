/*ckwg +5
 * Copyright 2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

/**
 * \file
 * \brief Implementation of \link maptk::vxl::close_loops
 *        close_loops \endlink
 */

#include <maptk/vxl/close_loops.h>
#include <maptk/vxl/compute_homography_overlap.h>
#include <maptk/core/algo/compute_ref_homography.h>

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
typedef homography_collection_sptr checkpoint_entry_t;

// Buffer type for detected checkpoints
typedef boost::circular_buffer< checkpoint_entry_t > checkpoint_buffer_t;


/// Private implementation class
class close_loops::priv
{
public:

  priv()
  : long_term_closure_enabled_( true ),
    max_checkpoint_frames_( 10000 ),
    checkpoint_percent_overlap_( 0.40 ),
    homography_filename_( "" )
  {
  }

  priv( const priv& other )
  : long_term_closure_enabled_( other.long_term_closure_enabled_ ),
    max_checkpoint_frames_( other.max_checkpoint_frames_ ),
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


close_loops
::close_loops()
: d_( new priv() )
{
}


close_loops
::close_loops( const close_loops& other )
: d_( new priv( *other.d_ ) )
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
  config_block_sptr config = algorithm::get_configuration();

  // Sub-algorithm implementation name + sub_config block
  // - Homography estimator algorithm
  maptk::algo::compute_ref_homography::get_nested_algo_configuration( "ref_computer", config, d_->ref_computer_ );

  // - Feature Matcher algorithm
  maptk::algo::match_features::get_nested_algo_configuration( "feature_matcher", config, d_->matcher_ );

  // Loop closure parameters
  config->set_value("long_term_closure_enabled", d_->long_term_closure_enabled_,
                    "Is long term loop closure enabled?");
  config->set_value("max_checkpoint_frames", d_->max_checkpoint_frames_,
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
  maptk::algo::compute_ref_homography_sptr rc;
  maptk::algo::compute_ref_homography::set_nested_algo_configuration( "ref_computer", config, rc );
  d_->ref_computer_ = rc;

  maptk::algo::match_features_sptr mf;
  maptk::algo::match_features::set_nested_algo_configuration( "feature_matcher", config, mf );
  d_->matcher_ = mf;

  // Settings for bad frame detection
  d_->long_term_closure_enabled_ = config->get_value<bool>( "long_term_closure_enabled" );
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
close_loops
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
close_loops
::stitch( frame_id_t frame_number,
          image_container_sptr image,
          track_set_sptr input ) const
{
  track_set_sptr updated_set = input;

  // Compute new homographies for this frame
  f2f_homography_sptr new_homography =
    d_->ref_computer_->estimate( frame_number, updated_set );

  // Write out homographies if enabled
  if( !d_->homography_filename_.empty() )
  {
    std::ofstream fout( d_->homography_filename_.c_str(), std::ios::app );
    fout << *(new_homography) << std::endl;
    fout.close();
  }

  // Determine if this is a new checkpoint frame
  // []

  // Perform matching to any past checkpoints we want to test
  // []

  // Return updated track set
  return updated_set;
}


} // end namespace vxl

} // end namespace maptk
