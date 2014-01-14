/*ckwg +5
 * Copyright 2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#ifndef MAPTK_ALGO_BUNDLE_ADJUST_H_
#define MAPTK_ALGO_BUNDLE_ADJUST_H_

#include <maptk/core/algo/algorithm.h>
#include <maptk/core/track_set.h>
#include <maptk/core/camera_map.h>
#include <maptk/core/landmark_map.h>

namespace maptk
{

namespace algo
{

/// An abstract base class for bundle adjustment using tracks
class MAPTK_CORE_EXPORT bundle_adjust
: public algorithm_def<bundle_adjust>
{
public:
  /// Return the name of this algorithm
  std::string type_name() const { return "bundle_adjust"; }

  /// Optimize the camera and landmark parameters given a set of tracks
  /**
   * \param [in,out] cameras the cameras to optimize
   * \param [in,out] landmarks the landmarks to optimize
   * \param [in] tracks the tracks to use as constraints
   */
  virtual void
  optimize(camera_map_sptr cameras,
           landmark_map_sptr landmarks,
           track_set_sptr tracks) const = 0;
};


} // end namespace algo

} // end namespace maptk


#endif // MAPTK_ALGO_BUNDLE_ADJUST_H_
