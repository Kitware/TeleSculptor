/*ckwg +5
 * Copyright 2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

/**
 * \file
 * \brief Implementation of OCV draw tracks algorithm
 */

#include <maptk/ocv/draw_tracks.h>
#include <maptk/ocv/ocv_algo_tools.h>
#include <maptk/ocv/image_container.h>

#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/format.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

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
  : draw_dual_display(true),
    draw_track_id(true),
    draw_untracked_features(true),
    draw_feature_lines(true),
    pattern("feature_tracks_%1%.png")
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
  bool draw_dual_display;
  bool draw_track_id;
  bool draw_untracked_features;
  bool draw_feature_lines;
  boost::format pattern;
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

  config->set_value( "draw_dual_display", d_->draw_dual_display,
                     "Draw the last image on the left, and the current image "
                     "on the right (for every frame)." );
  config->set_value( "draw_track_id", d_->draw_track_id,
                     "Draw track ids next to each feature point." );
  config->set_value( "draw_untracked_features", d_->draw_untracked_features,
                     "Draw untracked feature points in red." );
  config->set_value( "draw_feature_lines", d_->draw_feature_lines,
                     "Draw lines between tracked features on adj frames." );
  config->set_value( "pattern", "feature_tracks_%1%.png",
                     "The output pattern for drawn images." );

  return config;
}


/// Set this algorithm's properties via a config block
void
draw_tracks
::set_configuration(config_block_sptr in_config)
{
  config_block_sptr config = this->get_configuration();
  config->merge_config( in_config );

  d_->draw_dual_display = config->get_value<bool>( "draw_dual_display" );
  d_->draw_track_id = config->get_value<bool>( "draw_track_id" );
  d_->draw_untracked_features = config->get_value<bool>( "draw_untracked_features" );
  d_->draw_feature_lines = config->get_value<bool>( "draw_feature_lines" );
  d_->pattern = boost::format( config->get_value<std::string>( "pattern" ) );
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

  cv::Mat last_img;

  BOOST_FOREACH( image_container_sptr ctr_sptr, image_data )
  {
    // Paint active tracks on the input image
    cv::Mat img = ocv::image_container::maptk_to_ocv( ctr_sptr->get_image() );

    // Convert to 3 channel image if not one already
    if( img.channels() == 1 )
    {
      cv::cvtColor( img, img, CV_GRAY2BGR );
    }

    // Seed last image with this one
    if( !fid )
    {
      last_img = img;
    }

    // List of lines to draw on final image
    std::vector< std::pair< cv::Point, cv::Point > > lines;

    // Colors to use
    const cv::Scalar blue( 255, 0, 0 );
    const cv::Scalar red( 0, 0, 255 );
    const cv::Scalar yellow( 0, 255, 255 );

    // Draw points on input image
    BOOST_FOREACH( track_sptr trk, track_set->active_tracks( fid )->tracks() )
    {
      track::track_state ts = *( trk->find( fid ) );

      if( !ts.feat )
      {
        continue;
      }

      // Handle drawing the feature point on the image
      cv::Scalar color = blue;
      cv::Point loc( ts.feat->loc()[1], ts.feat->loc()[0] );
      cv::Point txt_offset( -1, 1 );
      std::string fid_str = boost::lexical_cast<std::string>( fid );

      if( trk->size() == 1 )
      {
        color = red;
      }
      else if( trk->first_frame() == fid )
      {
        color = yellow;
      }

      if( !d_->draw_untracked_features || trk->size() > 1 )
      {
        cv::circle( img, loc, 1, color, 1 );
      }

      if( d_->draw_track_id && trk->size() > 1 )
      {
        cv::putText( img, fid_str, loc + txt_offset, cv::FONT_HERSHEY_SIMPLEX, 3, color );
      }

      // Generate feature match line from the last image to this one
      if( d_->draw_feature_lines && trk->size() > 1 && fid > 0 )
      {
        track::history_const_itr itr = trk->find( fid-1 );

        if( itr != trk->end() && itr->feat )
        {
          cv::Point current_loc = loc + cv::Point( 0, img.cols );
          cv::Point prior_loc( itr->feat->loc()[1], itr->feat->loc()[0] );
          lines.push_back( std::make_pair( prior_loc, current_loc ) );
        }
      }
    }

    // Output image
    std::string ofn = boost::str( d_->pattern % fid );

    if( d_->draw_dual_display )
    {
      cv::Mat unioned_image( img.rows, img.cols + last_img.cols, img.type(), cv::Scalar(0) );

      cv::Mat left( unioned_image, cv::Rect( 0, 0, img.rows, img.cols ) );
      cv::Mat right( unioned_image, cv::Rect( 0, last_img.cols, img.rows, img.cols + last_img.cols) );

      last_img.copyTo( left );
      img.copyTo( right );

      for( unsigned i = 0; i < lines.size(); i++ )
      {
        cv::line( unioned_image, lines[i].first, lines[i].second, blue );
      }

      cv::imwrite( ofn.c_str(), unioned_image );
    }
    else
    {
      cv::imwrite( ofn.c_str(), img );
    }

    // Store last variables
    last_img = img;
  }
}


} // end namespace ocv

} // end namespace maptk
