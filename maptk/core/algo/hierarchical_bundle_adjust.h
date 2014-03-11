/*ckwg +5
 * Copyright 2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

/**
 * \file
 * \brief Header defining \link maptk::algo::hierarchical_bundle_adjust
 *        hierarchical bundle adjustment \endlink algorithm
 */

#ifndef MAPTK_ALGO_HIERARCHICAL_BUNDLE_ADJUST_H_
#define MAPTK_ALGO_HIERARCHICAL_BUNDLE_ADJUST_H_

#include <maptk/core/algo/algorithm.h>
#include <maptk/core/algo/bundle_adjust.h>
#include <maptk/core/config_block.h>
#include <maptk/core/core_config.h>

#include <boost/scoped_ptr.hpp>


namespace maptk
{

namespace algo
{


class MAPTK_CORE_EXPORT hierarchical_bundle_adjust
  : public algorithm_impl<hierarchical_bundle_adjust, bundle_adjust>
{
public:

  /// Constructor
  hierarchical_bundle_adjust();
  /// Copy constructor
  hierarchical_bundle_adjust(hierarchical_bundle_adjust const& other);
  /// Destructor
  virtual ~hierarchical_bundle_adjust() MAPTK_NOTHROW;

  /// Return the name of this implementation
  std::string impl_name() const { return "hierarchical"; }

  /// Get this algorithm's \link maptk::config_block configuration block \endlink
  virtual config_block_sptr get_configuration() const;
  /// Set this algorithm's properties via a config block
  virtual void set_configuration(config_block_sptr config);
  /// Check that the algorithm's configuration config_block is valid
  virtual bool check_configuration(config_block_sptr config) const;

  /// Optimize the camera and landmark parameters given a set of tracks
  virtual void optimize(camera_map_sptr & cameras,
                        landmark_map_sptr & landmarks,
                        track_set_sptr tracks) const;

private:
  // private implementation class
  class priv;
  boost::scoped_ptr<priv> d_;
};


/// Type definition for shared pointer for hierarchical_bundle_adjust algorithm
typedef boost::shared_ptr<hierarchical_bundle_adjust> hierarchical_bundle_adjust_sptr;


}

}


#endif // MAPTK_ALGO_HIERARCHICAL_BUNDLE_ADJUST_H_
