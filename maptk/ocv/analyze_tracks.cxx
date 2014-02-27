/*ckwg +5
 * Copyright 2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

/**
 * \file
 * \brief Implementation of OpenCV analyze tracks algorithm
 */

#include <maptk/ocv/analyze_tracks.h>
#include <maptk/ocv/ocv_algo_tools.h>

#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/format.hpp>

#include <set>
#include <fstream>
#include <iostream>
#include <iomanip>

namespace maptk
{

namespace ocv
{


/// Private implementation class
class analyze_tracks::priv
{
public:

  /// Constructor
  priv()
  : output_summary(true),
    output_pt_matrix(true),
    pt_matrix_cols(5)
  {
  }

  /// Copy Constructor
  priv(const priv& other)
  {
    *this = other;
  }

  /// Destructor
  ~priv()
  {
  }

  /// Text output parameters
  bool output_summary;
  bool output_pt_matrix;
  unsigned pt_matrix_cols;
};


/// Constructor
analyze_tracks
::analyze_tracks()
: d_(new priv)
{
}


/// Copy constructor
analyze_tracks
::analyze_tracks(const analyze_tracks& other)
: d_(new priv(*other.d_))
{
}


/// Destructor
analyze_tracks
::~analyze_tracks()
{
}


/// Get this algorithm's \link maptk::config_block configuration block \endlink
config_block_sptr
analyze_tracks
::get_configuration() const
{
  config_block_sptr config = maptk::algo::analyze_tracks::get_configuration();

  config->set_value("output_summary", d_->output_summary,
                    "Output a summary descriptor of high-level properties.");
  config->set_value("output_pt_matrix", d_->output_pt_matrix,
                    "Output a matrix showing details about the percentage of "
                    "features tracked for every frame, from each frame to the "
                    "last n frames before said frame.");
  config->set_value("pt_matrix_cols", d_->pt_matrix_cols,
                    "The number comparison frames for each frame to compute the "
                    "percent of features tracked statistics for.");

  return config;
}


/// Set this algorithm's properties via a config block
void
analyze_tracks
::set_configuration(config_block_sptr in_config)
{
  config_block_sptr config = this->get_configuration();
  config->merge_config( in_config );

  d_->output_summary = config->get_value<bool>( "output_summary" );
  d_->output_pt_matrix = config->get_value<bool>( "output_pt_matrix" );
  d_->pt_matrix_cols = config->get_value<unsigned>( "pt_matrix_cols" );
}


/// Check that the algorithm's currently configuration is valid
bool
analyze_tracks
::check_configuration(config_block_sptr config) const
{
  return true;
}


/// Output various information about the tracks stored in the input set
void
analyze_tracks
::print_info(track_set_sptr track_set,
             stream_t& stream) const
{
  // Constants
  const unsigned num_tracks = track_set->size();
  const frame_id_t first_frame = track_set->first_frame();
  const frame_id_t last_frame = track_set->last_frame();
  const frame_id_t total_frames = last_frame - first_frame + 1;

  // Output percent tracked matrix
  if( d_->output_pt_matrix )
  {
    stream << "        Percent of Features Tracked Matrix         " << std::endl;
    stream << "---------------------------------------------------" << std::endl;
    stream << "(FrameID) (NumTrks) (%TrkF-1) (%TrkF-2) (%TrkF-...)" << std::endl;
    stream << std::endl;
  }

  // Generate matrix
  cv::Mat_<double> data( total_frames, d_->pt_matrix_cols + 2 );

  for( unsigned fid = first_frame; fid <= last_frame; fid++ )
  {
    data.at<double>( fid, 0 ) = fid;
    data.at<double>( fid, 1 ) = track_set->active_tracks( fid )->size();

    for( unsigned c = 1; c <= d_->pt_matrix_cols; c++ )
    {
      if( fid < first_frame + c )
      {
        data.at<double>( fid, c+1 ) = -1.0;
      }
      else
      {
        data.at<double>( fid, c+1 ) = track_set->percentage_tracked( fid-c, fid );
      }
    }
  }

  // Output matrix if enabled
  if( d_->output_pt_matrix )
  {
    stream << data << std::endl;
  }

  // Output number of tracks in stream
  if( d_->output_summary )
  {
    stream << "Track Set Properties" << std::endl;
    stream << "--------------------" << std::endl;
    stream << std::endl;
    stream << "Largest Track ID: " << num_tracks << std::endl;
    stream << "Smallest Frame ID: " << first_frame << std::endl;
    stream << "Largest Frame ID: " << last_frame << std::endl;
    stream << std::endl;
  }
}


} // end namespace ocv

} // end namespace maptk
