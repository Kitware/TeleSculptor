/*ckwg +5
 * Copyright 2013 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#ifndef MAPTK_ALGO_TRACK_FEATURES_H_
#define MAPTK_ALGO_TRACK_FEATURES_H_

#include <maptk/core/algo/algorithm.h>
#include <maptk/core/algo/detect_features.h>
#include <maptk/core/algo/extract_descriptors.h>
#include <maptk/core/algo/match_features.h>
#include <maptk/core/image_container.h>
#include <maptk/core/track_set.h>

namespace maptk
{

namespace algo
{

/// An abstract base class for tracking feature points
class track_features : public algorithm_def<track_features>
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

  /// Set the feature detection algorithm to use in tracking
  void set_feature_detector(detect_features_sptr alg)
  {
    detector_ = alg;
  }

  /// Set the descriptor extraction algorithm to use in tracking
  void set_descriptor_extractor(extract_descriptors_sptr alg)
  {
    extractor_ = alg;
  }

  /// Set the feature matching algorithm to use in tracking
  void set_feature_matcher(match_features_sptr alg)
  {
    matcher_ = alg;
  }

protected:
  /// The feature detector algorithm to use
  detect_features_sptr detector_;

  /// The descriptor extractor algorithm to use
  extract_descriptors_sptr extractor_;

  /// The feature matching algorithm to use
  match_features_sptr matcher_;
};


/// A basic feature tracker
class simple_track_features
: public algo::algorithm_impl<simple_track_features, track_features>
{
public:
  /// Default Constructor
  simple_track_features() {}

  /// Copy Constructor
  simple_track_features(const simple_track_features&) {}

  /// Return the name of this implementation
  std::string impl_name() const { return "simple"; }

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
};


} // end namespace algo

} // end namespace maptk


#endif // MAPTK_ALGO_TRACK_FEATURES_H_
