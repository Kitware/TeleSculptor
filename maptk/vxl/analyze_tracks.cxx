/*ckwg +5
 * Copyright 2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

/**
 * \file
 * \brief Implementation of VXL analyze tracks algorithm
 */


#include <maptk/vxl/analyze_tracks.h>

#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>

#include <set>
#include <fstream>
#include <iostream>
#include <iomanip>

namespace maptk
{

namespace vxl
{


/// Private implementation class
class analyze_tracks::priv
{
public:

  /// Constructor
  priv()
  : output_sum_props(true),
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

  /// Parameters
  bool output_sum_props;
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
  const unsigned num_tracks = track_set->size();
  const frame_id_t first_frame = track_set->first_frame();
  const frame_id_t last_frame = track_set->last_frame();

  // Output number of tracks in stream
  if( d_->output_sum_props )
  {
    stream << "Track Set Properties" << std::endl;
    stream << "--------------------" << std::endl;
    stream << std::endl;
    stream << "Largest Track ID: " << num_tracks << std::endl;
    stream << "Smallest Frame ID: " << first_frame << std::endl;
    stream << "Largest Frame ID: " << last_frame << std::endl;
    stream << std::endl;
  }

  // Output percent tracked matrix
  if( d_->output_pt_matrix )
  {
    stream << "Percent of Features Tracked Matrix" << std::endl;
    stream << "----------------------------------" << std::endl;
    stream << std::endl;

    for( unsigned fid = first_frame; fid <= last_frame; fid++ )
    {
      stream << "Frame" << boost::lexical_cast<std::string>( fid );

      for( unsigned c = 1; c <= d_->pt_matrix_cols; c++ )
      {
        stream << " ";

        if( fid < first_frame + c )
        {
          stream << "----";
        }
        else
        {
          stream << std::setprecision(3) << track_set->percentage_tracked( fid-c, fid );
        }
      }

      stream << std::endl;
    }
  }
}


/// Output images with tracked features drawn on them
void
analyze_tracks
::write_images(track_set_sptr track_set,
               image_container_sptr_list image_data) const
{

}


} // end namespace vxl

} // end namespace maptk
