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
 *        tracking \endlink algorithm and concrete \link
 *        maptk::algo::simple_track_features simple_track_features \endlink
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

  /// Get this algorithm's \link maptk::config_block configuration block \endlink
  /**
   * This base virtual function implementation returns an empty configuration
   * block whose name is set to \c this->type_name.
   *
   * \returns \c config_block containing the configuration for this algorithm
   *          and any nested components.
   */
  virtual config_block_sptr get_configuration() const;

  /// Set this algorithm's properties via a config block
  /**
   * \throws no_such_configuration_value_exception
   *    Thrown if an expected configuration value is not present.
   * \throws algorithm_configuration_exception
   *    Thrown when the algorithm is given an invalid \c config_block or is'
   *    otherwise unable to configure itself.
   *
   * \param config  The \c config_block instance containing the configuration
   *                parameters for this algorithm
   */
  virtual void set_configuration(config_block_sptr config);

  /// Check that the algorithm's currently configuration is valid
  /**
   * This checks solely within the provided \c config_block and not against
   * the current state of the instance. This isn't static for inheritence
   * reasons.
   *
   * \param config  The config block to check configuration of.
   *
   * \returns true if the configuration check passed and false if it didn't.
   */
  virtual bool check_configuration(config_block_sptr config) const;

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


/// Shared pointer for generic track_features definition type.
typedef boost::shared_ptr<track_features> track_features_sptr;


/// A basic feature tracker
class MAPTK_CORE_EXPORT simple_track_features
  : public algo::algorithm_impl<simple_track_features, track_features>
{
public:
  /// Default Constructor
  simple_track_features();

  /// Copy Constructor
  simple_track_features(const simple_track_features&);

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

private:
  /// The ID to use for the next created track
  mutable unsigned long next_track_id_;
};


} // end namespace algo

} // end namespace maptk


#endif // MAPTK_ALGO_TRACK_FEATURES_H_
