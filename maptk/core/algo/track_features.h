/*ckwg +5
 * Copyright 2013-2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#ifndef MAPTK_ALGO_TRACK_FEATURES_H_
#define MAPTK_ALGO_TRACK_FEATURES_H_

#include <maptk/core/core_config.h>

#include <boost/shared_ptr.hpp>

#include <maptk/core/algo/algorithm.h>
#include <maptk/core/algo/detect_features.h>
#include <maptk/core/algo/extract_descriptors.h>
#include <maptk/core/algo/match_features.h>
#include <maptk/core/image_container.h>
#include <maptk/core/track_set.h>


/**
 * \file
 * \brief Header defining abstract \link maptk::algo::track_features feature
 *        tracking \endlink algorithm
 */

namespace maptk
{

namespace algo
{

/// An abstract base class for tracking feature points
class MAPTK_CORE_EXPORT track_features
  : public algorithm_def<track_features>
{
public:

  /// Return the name of this algorithm
  std::string type_name() const { return "track_features"; }

  /// Extend a previous set of tracks using the current frame
  /**
   * \param [in] prev_tracks the tracks from previous tracking steps
   * \param [in] frame_number the frame number of the current frame
   * \param [in] image_data the image pixels for the current frame
   * \returns an updated set a tracks including the current frame
   */
  virtual track_set_sptr
  track(track_set_sptr prev_tracks,
        unsigned int frame_number,
        image_container_sptr image_data) const = 0;

};


/// Shared pointer for generic track_features definition type.
typedef boost::shared_ptr<track_features> track_features_sptr;

} // end namespace algo

} // end namespace maptk


#endif // MAPTK_ALGO_TRACK_FEATURES_H_
