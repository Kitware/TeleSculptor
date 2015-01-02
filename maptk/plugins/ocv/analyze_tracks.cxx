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
 * \brief Implementation of OpenCV analyze tracks algorithm
 */

#include "analyze_tracks.h"

#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/format.hpp>

#include <fstream>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <set>
#include <vector>

#include <maptk/plugins/ocv/ocv_algo_tools.h>


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
    output_pt_matrix(true)
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
  std::vector<int> frames_to_compare;
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
                    "features tracked for every frame, from each frame to "
                    "some list of frames in the past.");
  config->set_value("frames_to_compare", "1, 5, 10, 50",
                    "A comma seperated list of frame difference intervals we want "
                    "to use for the pt matrix. For example, if \"1, 4\" the pt "
                    "matrix will contain comparisons between the current frame and "
                    "last frame in addition to four frames ago.");

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

  std::string ftc = config->get_value<std::string>( "frames_to_compare" );

  std::stringstream ss(ftc);
  d_->frames_to_compare.clear();

  int next_int;

  while (ss >> next_int)
  {
    d_->frames_to_compare.push_back(next_int);

    if (ss.peek() == ',')
    {
      ss.ignore();
    }
  }
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
  // Early exist case
  if( !d_->output_pt_matrix && !d_->output_summary )
  {
    return;
  }

  // Constants
  const unsigned num_tracks = static_cast<unsigned>(track_set->size());
  const frame_id_t first_frame = track_set->first_frame();
  const frame_id_t last_frame = track_set->last_frame();
  const frame_id_t total_frames = last_frame - first_frame + 1;

  // Output percent tracked matrix
  if( d_->output_pt_matrix )
  {
    stream << std::endl;
    stream << "        Percent of Features Tracked Matrix         " << std::endl;
    stream << "---------------------------------------------------" << std::endl;
    stream << "(FrameID) (NumTrks) (%TrkFromID ";

    for( unsigned i = 0; i < d_->frames_to_compare.size(); i++ )
    {
      stream << " -" << d_->frames_to_compare[i];
    }

    stream << ")" << std::endl;
    stream << std::endl;
  }

  // Generate matrix
  cv::Mat_<double> data( total_frames, static_cast<int>(d_->frames_to_compare.size()) + 2 );

  for( frame_id_t fid = first_frame; fid <= last_frame; fid++ )
  {
    data.at<double>( fid, 0 ) = fid;
    data.at<double>( fid, 1 ) = static_cast<double>(track_set->active_tracks( fid )->size());

    for( unsigned i = 0; i < d_->frames_to_compare.size(); i++ )
    {
      int adj = d_->frames_to_compare[ i ];

      if( fid < first_frame + adj )
      {
        data.at<double>( fid, i+2 ) = -1.0;
      }
      else
      {
        data.at<double>( fid, i+2 ) = track_set->percentage_tracked( fid-adj, fid );
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
    stream << std::endl;
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
