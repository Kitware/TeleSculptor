/*ckwg +5
* Copyright 2014 by Kitware, Inc. All Rights Reserved. Please refer to
* KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
* Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
*/

/**
* \file
* \brief Header defining VXL algorithm implementation of camera optimization.
*/

#ifndef MAPTK_VXL_OPTIMIZE_CAMERAS_H_
#define MAPTK_VXL_OPTIMIZE_CAMERAS_H_

#include <string>

#include <maptk/core/algo/algorithm.h>
#include <maptk/core/algo/optimize_cameras.h>
#include <maptk/core/config_block.h>
#include <maptk/vxl/vxl_config.h>


namespace maptk
{

namespace vxl
{


class MAPTK_VXL_EXPORT optimize_cameras
  : public algo::algorithm_impl<optimize_cameras, algo::optimize_cameras>
{
public:
  std::string impl_name() const { return "vxl"; }

  /// \cond DoxygenSuppress
  virtual void set_configuration(config_block_sptr /*config*/) { }
  virtual bool check_configuration(config_block_sptr /*config*/) const { return true; }
  /// \endcond

  /// Optimize camera parameters given sets of landmarks and tracks
  /**
   * We only optimize cameras that have associating tracks and landmarks in
   * the given maps.
   *
   * \throws invalid_value When one or more of the given pointer is Null.
   *
   * \param[in,out] cameras   Cameras to optimize.
   * \param[in]     tracks    The tracks to use as constraints.
   * \param[in]     landmarks The landmarks the cameras are viewing.
   */
  virtual void
  optimize(camera_map_sptr & cameras,
           track_set_sptr tracks,
           landmark_map_sptr landmarks) const;
};


}

}

#endif // MAPTK_VXL_OPTIMIZE_CAMERAS_H_
