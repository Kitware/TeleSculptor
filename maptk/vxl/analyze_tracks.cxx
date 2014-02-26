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

#include <set>
#include <fstream>


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
  {
  }

  /// Copy Constructor
  priv(const priv& other)
  {
  }

  /// Destructor
  ~priv()
  {
  }

  /// parameters - none yet
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
  // get base config from base class
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
::analyze(track_set_sptr track_set) const
{

}


/// Output various information about the tracks stored in the input set
void
analyze_tracks
::analyze(track_set_sptr track_set,
          image_container_sptr_list image_data) const
{
  /// Output any data not using image contents
  this->analyze(track_set);
}


} // end namespace vxl

} // end namespace maptk
