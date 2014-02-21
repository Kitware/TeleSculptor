/*ckwg +5
 * Copyright 2013-2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#ifndef MAPTK_ALGO_TRACK_FEATURES_DEFAULT_H_
#define MAPTK_ALGO_TRACK_FEATURES_DEFAULT_H_

#include <maptk/core/core_config.h>

#include <boost/shared_ptr.hpp>

#include <maptk/core/algo/algorithm.h>
#include <maptk/core/algo/detect_features.h>
#include <maptk/core/algo/extract_descriptors.h>
#include <maptk/core/algo/match_features.h>
#include <maptk/core/algo/track_features.h>
#include <maptk/core/image_container.h>
#include <maptk/core/track_set.h>


/**
 * \file
 * \brief Header defining the \link maptk::algo::track_features_default
 *        track_features_default \endlink algorithm
 */

namespace maptk
{

namespace algo
{

/// A basic feature tracker
class MAPTK_CORE_EXPORT track_features_default
  : public algo::algorithm_impl<track_features_default, track_features>
{
public:

  /// Default Constructor
  track_features_default();

  /// Copy Constructor
  track_features_default(const track_features_default&);

  /// Return the name of this implementation
  std::string impl_name() const { return "default"; }

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
        image_container_sptr image_data) const;

private:

  /// The ID to use for the next created track
  mutable unsigned long next_track_id_;
};


} // end namespace algo

} // end namespace maptk


#endif // MAPTK_ALGO_TRACK_FEATURES_H_
