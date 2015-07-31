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
 * \brief Implementation of OCV draw tracks algorithm
 */

#include "draw_tracks.h"

#include <set>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <algorithm>

#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/format.hpp>
#include <boost/circular_buffer.hpp>
#include <boost/filesystem.hpp>

#include <maptk/plugins/ocv/image_container.h>
#include <maptk/plugins/ocv/ocv_algo_tools.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace kwiver::vital;

namespace kwiver {
namespace maptk {

namespace ocv
{


/// Helper typedef for storing match lines between frames
typedef std::vector< std::pair< cv::Point, cv::Point > > line_vec_t;


/// Helper typedef for storing past frame id offsets
typedef std::vector< frame_id_t > fid_offset_vec_t;


/// Private implementation class
class draw_tracks::priv
{
public:

  /// Constructor
  priv()
  : draw_track_ids( true ),
    draw_untracked_features( true ),
    draw_match_lines( false ),
    draw_shift_lines( false ),
    draw_comparison_lines( true ),
    swap_comparison_set( false ),
    write_images_to_disk( true ),
    pattern( "feature_tracks_%05d.png" ),
    cur_frame_id( 0 )
  {
  }

  /// Copy Constructor
  priv( const priv& other )
  {
    *this = other;
  }

  /// Destructor
  ~priv()
  {
  }

  /// Parameters
  bool draw_track_ids;
  bool draw_untracked_features;
  bool draw_match_lines;
  bool draw_shift_lines;
  bool draw_comparison_lines;
  bool swap_comparison_set;
  fid_offset_vec_t past_frames_to_show;
  bool write_images_to_disk;
  boost::format pattern;

  /// Internal variables
  boost::circular_buffer< cv::Mat > buffer;
  frame_id_t cur_frame_id;
};


/// Constructor
draw_tracks
::draw_tracks()
: d_( new priv )
{
}


/// Copy constructor
draw_tracks
::draw_tracks( const draw_tracks& other )
: d_( new priv( *other.d_ ) )
{
}


/// Destructor
draw_tracks
::~draw_tracks()
{
}


/// Get this algorithm's \link kwiver::vital::config_block configuration block \endlink
kwiver::vital::config_block_sptr
draw_tracks
::get_configuration() const
{
  kwiver::vital::config_block_sptr config = kwiver::vital::algo::draw_tracks::get_configuration();

  config->set_value( "draw_track_ids", d_->draw_track_ids,
                     "Draw track ids next to each feature point." );
  config->set_value( "draw_untracked_features", d_->draw_untracked_features,
                     "Draw untracked feature points in error_color." );
  config->set_value( "draw_match_lines", d_->draw_match_lines,
                     "Draw lines between tracked features on the current frame "
                     "to any past frames." );
  config->set_value( "draw_shift_lines", d_->draw_shift_lines,
                     "Draw lines showing the movement of the feature in the image "
                     "plane from the last frame to the current one drawn on every "
                     "single image individually." );
  config->set_value( "draw_comparison_lines", d_->draw_comparison_lines,
                     "If more than 1 track set is input to this class, should we "
                     "draw comparison lines between tracks with the same ids in "
                     "both input sets?" );
  config->set_value( "swap_comparison_set", d_->swap_comparison_set,
                     "If we are using a comparison track set, swap it and the input "
                     "track set, so that the comparison set becomes the main set "
                     "being displayed." );
  config->set_value( "past_frames_to_show", "",
                     "A comma seperated list of past frames to show. For example: "
                     "a value of \"2, 1\" will cause the GUI to generate a window "
                     "3 frames wide, with the first frame being 2 frames behind the "
                     "current frame, the second 1 frame behind, and the third being "
                     "the current frame." );
  config->set_value( "write_images_to_disk", "true",
                     "Should images be written out to disk?" );
  config->set_value( "pattern", "feature_tracks_%1%.png",
                     "The output pattern for writing images to disk." );

  return config;
}


/// Set this algorithm's properties via a config block
void
draw_tracks
::set_configuration( kwiver::vital::config_block_sptr in_config )
{
  kwiver::vital::config_block_sptr config = this->get_configuration();
  config->merge_config( in_config );

  std::string past_frames_str = config->get_value<std::string>( "past_frames_to_show" );

  std::stringstream ss( past_frames_str );
  d_->past_frames_to_show.clear();

  unsigned next_int;
  while( ss >> next_int )
  {
    d_->past_frames_to_show.push_back( next_int );

    if( ss.peek() == ',' )
    {
      ss.ignore();
    }
  }

  d_->draw_track_ids = config->get_value<bool>( "draw_track_ids" );
  d_->draw_untracked_features = config->get_value<bool>( "draw_untracked_features" );
  d_->draw_match_lines = config->get_value<bool>( "draw_match_lines" );
  d_->draw_shift_lines = config->get_value<bool>( "draw_shift_lines" );
  d_->draw_comparison_lines = config->get_value<bool>( "draw_comparison_lines" );
  d_->swap_comparison_set = config->get_value<bool>( "swap_comparison_set" );
  d_->write_images_to_disk = config->get_value<bool>( "write_images_to_disk" );
  d_->pattern = boost::format( config->get_value<std::string>( "pattern" ) );

  if( !d_->past_frames_to_show.empty() )
  {
    d_->buffer.set_capacity( *std::max_element( d_->past_frames_to_show.begin(),
                                                d_->past_frames_to_show.end()) );
  }
}


/// Check that the algorithm's currently configuration is valid
bool
draw_tracks
::check_configuration(kwiver::vital::config_block_sptr config) const
{
  return true;
}


/// Helper function to subtract a value from all elements in a vec
void subtract_from_all( fid_offset_vec_t& offsets, unsigned value )
{
  for( unsigned i = 0; i < offsets.size(); i++ )
  {
    offsets[i] -= value;
  }
}


/// Helper function to convert a track state to an OpenCV point
cv::Point state_to_cv_point( const track::track_state& ts )
{
  return cv::Point( static_cast<int>(ts.feat->loc()[0]),
                    static_cast<int>(ts.feat->loc()[1]) );
}


/// Helper function for creating valid match lines for a given track
void generate_match_lines( const track_sptr trk,
                           const frame_id_t frame_id,
                           const fid_offset_vec_t& frame_offsets,
                           const cv::Point& image_offset,
                           line_vec_t& line_list )
{
  if( frame_offsets.empty() )
  {
    return;
  }

  track::history_const_itr frame_itr = trk->find( frame_id );

  if( frame_itr == trk->end() || !frame_itr->feat )
  {
    return;
  }

  const cv::Point frame_loc( static_cast<int>(frame_itr->feat->loc()[0]),
                             static_cast<int>(frame_itr->feat->loc()[1]) );

  fid_offset_vec_t rem_offsets = frame_offsets;

  for( unsigned i = 0; i < frame_offsets.size(); i++ )
  {
    const unsigned offset_to_test = rem_offsets.back();
    rem_offsets.pop_back();

    if( offset_to_test && frame_id >= offset_to_test )
    {
      const frame_id_t test_frame_id = frame_id - offset_to_test;
      track::history_const_itr test_itr = trk->find( test_frame_id );

      if( test_itr != trk->end() && test_itr->feat )
      {
        // add line
        cv::Point test_loc = state_to_cv_point( *test_itr );
        cv::Point frame_offset = static_cast<int>( frame_offsets.size() ) * image_offset;
        cv::Point test_offset = static_cast<int>( frame_offsets.size() - i - 1 ) * image_offset;
        line_list.push_back( std::make_pair( frame_loc + frame_offset, test_loc + test_offset ) );

        // call this function recursively to pick up anymore lines
        subtract_from_all( rem_offsets, offset_to_test );
        generate_match_lines( trk, test_frame_id, rem_offsets, image_offset, line_list );

        // break out of loop
        return;
      }
    }
  }
}


/// Output images with tracked features drawn on them
image_container_sptr
draw_tracks
::draw( track_set_sptr input_display_set,
        image_container_sptr_list image_data,
        track_set_sptr input_comparison_set )
{
  namespace bfs = boost::filesystem;

  // Perform swap of inputs if settings enabled
  track_set_sptr display_set = input_display_set;
  track_set_sptr comparison_set = input_comparison_set;

  if( d_->swap_comparison_set )
  {
    std::swap( display_set, comparison_set );
  }

  // Validate inputs
  if( image_data.empty() )
  {
    std::cerr << "Error: valid imagery must be provided" << std::endl;
    return image_container_sptr();
  }

  // Has a valid comparison track set been provided?
  const bool comparison_set_provided = comparison_set && !comparison_set->empty();

  // The output image
  cv::Mat output_image;

  // The total number of past frames we are showing
  const unsigned past_frames = static_cast<unsigned>( d_->past_frames_to_show.size() );

  // The total number of output frames to display
  const unsigned display_frames = past_frames + 1;

  // Generate output images
  frame_id_t fid = d_->cur_frame_id;

  // Colors to use
  const cv::Scalar default_color( 255, 0, 0 );
  const cv::Scalar new_color( 0, 255, 0 );
  const cv::Scalar terminated_color( 240, 32, 160 );
  const cv::Scalar untracked_color( 0, 69, 255 );
  const cv::Scalar error_color( 0, 0, 255 );
  const cv::Scalar uncompared_color( 240, 32, 160 );

  // Iterate over all images
  BOOST_FOREACH( image_container_sptr ctr_sptr, image_data )
  {
    // Should the current frame be written to disk?
    bool write_image_to_disk = d_->write_images_to_disk;

    // Clone a copy of the current image (so we don't modify the original input).
    cv::Mat img = ocv::image_container::maptk_to_ocv( ctr_sptr->get_image() ).clone();

    // Convert to 3 channel image if not one already
    if( img.channels() == 1 )
    {
      cv::cvtColor( img, img, CV_GRAY2BGR );
    }

    // List of match lines to draw on final image
    line_vec_t lines;

    // Adjustment added to bring a point to a seperate window
    const cv::Point pt_adj( img.cols, 0 );

    // Has at least one comparison track been found for this frame?
    bool comparison_track_found = false;

    // Draw points on input image
    BOOST_FOREACH( track_sptr trk, display_set->active_tracks( fid )->tracks() )
    {
      track::track_state ts = *( trk->find( fid ) );

      if( !ts.feat )
      {
        continue;
      }

      // Handle drawing the feature point on the image
      cv::Scalar color = default_color;
      cv::Point loc = state_to_cv_point( ts );
      cv::Point txt_offset( 2, -2 );
      std::string tid_str = boost::lexical_cast<std::string>( trk->id() );

      if( trk->size() == 1 )
      {
        color = untracked_color;
      }
      else if( trk->first_frame() == fid )
      {
        color = new_color;
      }
      else if( trk->last_frame() == fid )
      {
        color = terminated_color;
      }

      // Generate and store match lines for later use
      if( d_->draw_match_lines )
      {
        generate_match_lines( trk, fid, d_->past_frames_to_show, pt_adj, lines );
      }

      // Generate comparison lines
      if( d_->draw_comparison_lines && comparison_set_provided )
      {
        track_sptr comparison_trk = comparison_set->get_track( trk->id() );

        if( comparison_trk )
        {
          track::history_const_itr itr = comparison_trk->find( fid );

          if( itr != comparison_trk->end() && itr->feat )
          {
            cv::Point other_loc = state_to_cv_point( *itr );
            cv::line( img, other_loc, loc, error_color, 2 );
            comparison_track_found = true;
          }
        }
        else
        {
          color = uncompared_color;
        }
      }

      // Generate and draw shift lines on the video
      if( d_->draw_shift_lines && trk->size() > 1 && fid > 0 )
      {
        track::history_const_itr itr = trk->find( fid-1 );

        if( itr != trk->end() && itr->feat )
        {
          cv::Point prior_loc = state_to_cv_point( *itr );
          cv::line( img, prior_loc, loc, color );
        }
      }

      if( d_->draw_track_ids && trk->size() > 1 )
      {
        cv::putText( img, tid_str, loc + txt_offset, cv::FONT_HERSHEY_COMPLEX_SMALL, 0.5, color );
      }

      if( d_->draw_untracked_features || trk->size() > 1 )
      {
        cv::circle( img, loc, 1, color, 3 );
      }
    }

    // Update write image flag
    write_image_to_disk &= ( !comparison_set_provided || comparison_track_found );

    // Fully generate and output the image
    std::string ofn = boost::str( d_->pattern % fid );

    output_image = cv::Mat( img.rows, display_frames*img.cols, img.type(), cv::Scalar(0) );

    for( unsigned i = 0; i < past_frames; i++ )
    {
      cv::Mat region( output_image, cv::Rect( i*img.cols, 0, img.cols, img.rows ) );

      if( ((signed) d_->buffer.size() >= d_->past_frames_to_show[i]) &&
          (d_->past_frames_to_show[i] != 0) )
      {
        d_->buffer[ d_->buffer.size()-d_->past_frames_to_show[i] ].copyTo( region );
      }
    }

    cv::Mat region( output_image, cv::Rect( past_frames*img.cols, 0, img.cols, img.rows ) );
    img.copyTo( region );

    for( unsigned i = 0; i < lines.size(); i++ )
    {
      cv::line( output_image, lines[i].first, lines[i].second, default_color );
    }

    if( write_image_to_disk )
    {
      // Check that the directory of the given filepath exists, creating necessary
      // directories where needed.
      path_t parent_dir = bfs::absolute(path_t(ofn).parent_path());
      if(!bfs::is_directory(parent_dir))
      {
        if(!bfs::create_directories(parent_dir))
        {
          throw file_write_exception(parent_dir, "Attempted directory creation, "
                                                 "but no directory created! No "
                                                 "idea what happened here...");
        }
      }
      cv::imwrite( ofn.c_str(), output_image );
    }

    // Store last image with all features and shift lines already drawn on it
    if( d_->buffer.capacity() > 0 )
    {
      d_->buffer.push_back( img );
    }

    // Increase frame id counter
    fid++;
  }

  // Store latest states
  d_->cur_frame_id = fid;

  // Return the last generated image
  return image_container_sptr( new ocv::image_container( output_image ) );
}


} // end namespace ocv

} // end namespace maptk
} // end namespace kwiver
