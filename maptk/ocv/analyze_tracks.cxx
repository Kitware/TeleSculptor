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
#include <maptk/ocv/image_container.h>

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
  : output_sum_props(true),
    output_pt_matrix(true),
    pt_matrix_cols(5),
    image_pattern("feature_image_%1%.png")
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
  bool output_sum_props;
  bool output_pt_matrix;
  unsigned pt_matrix_cols;

  /// Image output parameters
  boost::format image_pattern;
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
  return config;
}


/// Set this algorithm's properties via a config block
void
analyze_tracks
::set_configuration(config_block_sptr in_config)
{
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

  // Counters
  double summed_pt = 0.0;

  // Output percent tracked matrix
  if( d_->output_pt_matrix )
  {
    stream << "Percent of Features Tracked Matrix" << std::endl;
    stream << "----------------------------------" << std::endl;
    stream << "[FrameID] [NumTrks] [%TrkFromLast]" << std::endl;
    stream << std::endl;
  }

  for( unsigned fid = first_frame; fid <= last_frame; fid++ )
  {
    std::string fid_str = "Frame" + boost::lexical_cast<std::string>( fid );

    while( fid_str.size() < 12 )
    {
      fid_str = fid_str + " ";
    }
    for( unsigned c = 1; c <= d_->pt_matrix_cols; c++ )
    {
      stream << " ";
      if( fid < first_frame + c )
      {
        stream << "----";
      }
      else
      {
        double ptracked = track_set->percentage_tracked( fid-c, fid );
        stream << std::setprecision(3) << ptracked;
      }
    }
    stream << std::endl;
  }

  // Output number of tracks in stream
  if( d_->output_sum_props )
  {
    stream << "Track Set Properties" << std::endl;
    stream << "--------------------" << std::endl;
    stream << std::endl;
    stream << "Largest Track ID: " << num_tracks << std::endl;
    stream << "Smallest Frame ID: " << first_frame << std::endl;
    stream << "Largest Frame ID: " << last_frame << std::endl;
    stream << "Averaged %Tracked: " << summed_pt / total_frames << std::endl;
    stream << std::endl;
  }
}


/// Output images with tracked features drawn on them
void
analyze_tracks
::write_images(track_set_sptr track_set,
               image_container_sptr_list image_data) const
{
  // Validate inputs
  if( image_data.size() < track_set->last_frame() )
  {
    std::cerr << "Error: not enough imagery to display all tracks" << std::endl;
  }

  // Generate output images
  frame_id_t fid = 0;

  BOOST_FOREACH( image_container_sptr ctr_sptr, image_data )
  {
    // Paint active tracks on the input image
    cv::Mat img = ocv::image_container::maptk_to_ocv( ctr_sptr->get_image() );

    // Draw points on input image
    BOOST_FOREACH( track_sptr trk, track_set->active_tracks( fid )->tracks() )
    {
      track::track_state ts = *( trk->find( fid ) );

      if( !ts.feat )
      {
        continue;
      }

      cv::Scalar color( 255, 0, 0 );
      cv::Point loc( ts.feat->loc()[0], ts.feat->loc()[1] );
      cv::Point txt_offset( -1, 1 );
      std::string fid_str = boost::lexical_cast<std::string>( fid );

      if( trk->size() == 1 )
      {
        color = cv::Scalar( 0, 0, 255 );
      }

      cv::circle( img, loc, 1, color, 1 );
      cv::putText( img, fid_str, loc + txt_offset, cv::FONT_HERSHEY_SIMPLEX, 3, color );
    }

    // Output image
    std::string ofn = boost::str( d_->image_pattern % fid );
  }
}


} // end namespace ocv

} // end namespace maptk
