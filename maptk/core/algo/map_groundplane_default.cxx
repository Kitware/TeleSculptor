/*ckwg +5
 * Copyright 2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

/**
 * \file
 * \brief Implementation of \link maptk::algo::map_groundplane_default
 *        map_groundplane_default \endlink
 */

#include "map_groundplane_default.h"

#include <maptk/core/algo/estimate_homography.h>

#include <algorithm>
#include <iostream>
#include <set>
#include <vector>

#include <boost/foreach.hpp>


namespace maptk
{

namespace algo
{


// Extra data stored for every active track
struct extra_track_info
{
  // Location of this track in the reference frame
  homography_point ref_loc;

  // Is the ref loc valid?
  bool ref_loc_valid;

  // Should this point be used in homography regression?
  bool is_good;

  // The number of times we haven't seen this track as active
  unsigned missed_count;

  // Pointer to the track object this class extends
  track_sptr trk;

  // Constructor.
  extra_track_info()
  : ref_loc( 0.0, 0.0 ),
    ref_loc_valid( false ),
    is_good( true ),
    missed_count( 0 )
  {}
};


// Buffer type for the extra track info
typedef std::vector< extra_track_info > track_ext_buffer_t;

// Internal homography estimator type
typedef maptk::algo::estimate_homography_sptr estimator_sptr;


// Private implementation class
class map_groundplane_default::priv
{
public:

  priv()
  : some_double( 0.0 )
  {
  }

  priv( const priv& other )
  : some_double( other.some_double )
  {
  }

  ~priv()
  {
  }

  /// Some double
  double some_double;

  /// Buffer storing track extensions
  track_ext_buffer_t buffer_;

  /// Pointer to homography estimator
  estimator_sptr h_estimator_;
};


map_groundplane_default
::map_groundplane_default()
: d_( new priv() )
{
}


map_groundplane_default
::map_groundplane_default( const map_groundplane_default& other )
: d_( new priv( *other.d_ ) )
{
}


map_groundplane_default
::~map_groundplane_default()
{
}


config_block_sptr
map_groundplane_default
::get_configuration() const
{
  // get base config from base class
  config_block_sptr config = algorithm::get_configuration();

  // Sub-algorithm implementation name + sub_config block
  // - Homography estimator algorithm
  estimate_homography::get_nested_algo_configuration
    ( "homography_estimator", config, d_->h_estimator_ );

  return config;
}


void
map_groundplane_default
::set_configuration( config_block_sptr in_config )
{
  // Starting with our generated config_block to ensure that assumed values are present
  // An alternative is to check for key presence before performing a get_value() call.
  config_block_sptr config = this->get_configuration();
  config->merge_config( in_config );

  // Setting nested algorithm instances via setter methods instead of directly
  // assigning to instance property.
  estimate_homography::set_nested_algo_configuration
    ( "homography_estimator", config, d_->h_estimator_ );
}


bool
map_groundplane_default
::check_configuration(config_block_sptr config) const
{
  return
    estimate_homography::check_nested_algo_configuration
      ( "homography_estimator", config );
}


homography_collection_sptr
map_groundplane_default
::transform( frame_id_t frame_number,
             track_set_sptr tracks ) const
{

}

} // end namespace algo

} // end namespace maptk
