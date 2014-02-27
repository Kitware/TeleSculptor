/*ckwg +5
 * Copyright 2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

/**
 * \file
 * \brief Header for VXL bundle adjustment algorithm
 */

#ifndef MAPTK_VXL_BUNDLE_ADJUST_H_
#define MAPTK_VXL_BUNDLE_ADJUST_H_

#include <maptk/vxl/vxl_config.h>

#include <maptk/core/algo/bundle_adjust.h>
#include <boost/scoped_ptr.hpp>

namespace maptk
{

namespace vxl
{

/// A class for bundle adjustment of tracks using VXL
class MAPTK_VXL_EXPORT bundle_adjust
: public algo::algorithm_impl<bundle_adjust, algo::bundle_adjust>
{
public:
  /// Constructor
  bundle_adjust();

  /// Destructor
  virtual ~bundle_adjust();

  /// Copy Constructor
  bundle_adjust(const bundle_adjust& other);

  /// Return the name of this implementation
  std::string impl_name() const { return "vxl"; }

  /// Get this algorithm's \link maptk::config_block configuration block \endlink
  virtual config_block_sptr get_configuration() const;
  /// Set this algorithm's properties via a config block
  virtual void set_configuration(config_block_sptr config);
  /// Check that the algorithm's currently configuration is valid
  virtual bool check_configuration(config_block_sptr config) const;

  /// Optimize the camera and landmark parameters given a set of tracks
  /**
   * \param [in,out] cameras the cameras to optimize
   * \param [in,out] landmarks the landmarks to optimize
   * \param [in] tracks the tracks to use as constraints
   */
  virtual void
  optimize(camera_map_sptr& cameras,
           landmark_map_sptr& landmarks,
           track_set_sptr tracks) const;

private:
  /// private implementation class
  class priv;
  boost::scoped_ptr<priv> d_;
};


} // end namespace vxl

} // end namespace maptk


#endif // MAPTK_VXL_BUNDLE_ADJUST_H_
