/*ckwg +5
 * Copyright 2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

/**
 * \file
 * \brief Header defining abstract \link maptk::algo::optimize_cameras camera
 *        optimization \endlink algorithm
 */

#ifndef MAPTK_CORE_ALGO_OPTIMIZE_CAMERAS_H_
#define MAPTK_CORE_ALGO_OPTIMIZE_CAMERAS_H_

#include <maptk/core/algo/algorithm.h>
#include <maptk/core/camera_map.h>
#include <maptk/core/landmark_map.h>
#include <maptk/core/track_set.h>


namespace maptk
{

namespace algo
{


/// Abstract algorithm definition base for optimizing cameras
class MAPTK_CORE_EXPORT optimize_cameras
  : public algorithm_def<optimize_cameras>
{
public:
  /// Return the name of this algorithm definition
  std::string type_name() const { return "optimize_cameras"; }

  /// Optimize camera parameters given sets of landmarks and tracks
  /**
   * We only optimize cameras that have associating tracks and landmarks in
   * the given maps.
   *
   * \param[in,out] cameras   Cameras to optimize.
   * \param[in]     tracks    The tracks to use as constraints.
   * \param[in]     landmarks The landmarks the cameras are viewing.
   */
  virtual void
  optimize(camera_map_sptr & cameras,
           track_set_sptr tracks,
           landmark_map_sptr landmarks) const = 0;

};


}

}


#endif // MAPTK_CORE_ALGO_OPTIMIZE_CAMERAS_H_
