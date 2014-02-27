/*ckwg +5
 * Copyright 2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

/**
 * \file
 * \brief Header defining the \link maptk::algo::close_loops_stitcher_only
 *        close_loops_stitcher_only \endlink algorithm
 */

#ifndef MAPTK_ALGO_CLOSE_LOOPS_STITCHER_ONLY_H_
#define MAPTK_ALGO_CLOSE_LOOPS_STITCHER_ONLY_H_

#include <maptk/core/core_config.h>

#include <boost/shared_ptr.hpp>

#include <maptk/core/algo/algorithm.h>
#include <maptk/core/algo/match_features.h>
#include <maptk/core/algo/close_loops.h>
#include <maptk/core/image_container.h>
#include <maptk/core/track_set.h>


namespace maptk
{

namespace algo
{

/// A basic close_loops implementation which aims at simply stitching
/// over periods of "bad frames" in the input data.
class MAPTK_CORE_EXPORT close_loops_stitcher_only
  : public algo::algorithm_impl<close_loops_stitcher_only, close_loops>
{
public:

  /// Default Constructor
  close_loops_stitcher_only();

  /// Copy Constructor
  close_loops_stitcher_only(const close_loops_stitcher_only&);

  /// Destructor
  virtual ~close_loops_stitcher_only() {}

  /// Return the name of this implementation
  std::string impl_name() const { return "shot_stitcher_only"; }

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

  /// Perform basic shot stitching for bad frame detection
  /**
   * \param [in] frame_number the frame number of the current frame
   * \param [in] input the input track set to stitch
   * \returns an updated set a tracks after the stitching operation
   */
  virtual track_set_sptr
  stitch( frame_id_t frame_number,
          track_set_sptr input ) const;

protected:

  /// Is frame stitching enabled?
  bool stitching_enabled_;

  /// Stitching percent feature match required
  double stitching_percent_match_req_;

  /// Stitching required new valid shot size in frames
  unsigned stitching_new_shot_length_;

  /// Max search length for stitching in frames
  unsigned stitching_max_search_length_;

  /// The feature matching algorithm to use
  match_features_sptr matcher_;

};


} // end namespace algo

} // end namespace maptk


#endif // MAPTK_ALGO_CLOSE_LOOPS_STITCHER_ONLY_H_
