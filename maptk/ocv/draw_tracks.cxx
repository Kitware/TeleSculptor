/*ckwg +5
 * Copyright 2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

/**
 * \file
 * \brief Implementation of OpenCV draw tracks algorithm
 */

#include <maptk/ocv/draw_tracks.h>
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
class draw_tracks::priv
{
public:

  /// Constructor
  priv()
  : output_dual_display(true),
    output_text_track_id(true),
    output_untracked_features(true),
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

  /// Parameters
  bool output_dual_display;
  bool output_text_track_id;
  bool output_untracked_features;
  boost::format image_pattern;
};


/// Constructor
draw_tracks
::draw_tracks()
: d_(new priv)
{
}


/// Copy constructor
draw_tracks
::draw_tracks(const draw_tracks& other)
: d_(new priv(*other.d_))
{
}


/// Destructor
draw_tracks
::~draw_tracks()
{
}


/// Get this algorithm's \link maptk::config_block configuration block \endlink
config_block_sptr
draw_tracks
::get_configuration() const
{
  config_block_sptr config = maptk::algo::draw_tracks::get_configuration();
  return config;
}


/// Set this algorithm's properties via a config block
void
draw_tracks
::set_configuration(config_block_sptr in_config)
{
}


/// Check that the algorithm's currently configuration is valid
bool
draw_tracks
::check_configuration(config_block_sptr config) const
{
  return true;
}


/// Output images with tracked features drawn on them
void
draw_tracks
::draw(track_set_sptr track_set,
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
