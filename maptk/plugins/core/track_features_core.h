/*ckwg +29
 * Copyright 2013-2016 by Kitware, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither name of Kitware, Inc. nor the names of any contributors may be used
 *    to endorse or promote products derived from this software without specific
 *    prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS ``AS IS''
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHORS OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * \file
 * \brief Header defining the \link maptk::algo::track_features_core
 *        track_features_core \endlink algorithm
 */

#ifndef MAPTK_PLUGINS_CORE_TRACK_FEATURES_CORE_H_
#define MAPTK_PLUGINS_CORE_TRACK_FEATURES_CORE_H_


#include <vital/vital_config.h>
#include <maptk/plugins/core/maptk_core_export.h>

#include <vital/algo/algorithm.h>
#include <vital/algo/track_features.h>
#include <vital/types/image_container.h>
#include <vital/types/track_set.h>

#include <vital/algo/detect_features.h>
#include <vital/algo/extract_descriptors.h>
#include <vital/algo/match_features.h>
#include <vital/algo/close_loops.h>


namespace kwiver {
namespace maptk {

namespace core
{

/// A basic feature tracker
class MAPTK_CORE_EXPORT track_features_core
  : public vital::algorithm_impl<track_features_core, vital::algo::track_features>
{
public:

  /// Default Constructor
  track_features_core();

  /// Copy Constructor
  track_features_core(const track_features_core&);

  /// Return the name of this implementation
  virtual std::string impl_name() const { return "core"; }

  /// Get this algorithm's \link vital::config_block configuration block \endlink
  /**
   * This base virtual function implementation returns an empty configuration
   * block whose name is set to \c this->type_name.
   *
   * \returns \c config_block containing the configuration for this algorithm
   *          and any nested components.
   */
  virtual vital::config_block_sptr get_configuration() const;

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
  virtual void set_configuration(vital::config_block_sptr config);

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
  virtual bool check_configuration(vital::config_block_sptr config) const;

  /// Extend a previous set of tracks using the current frame
  /**
   * \throws image_size_mismatch_exception
   *    When the given non-zero mask image does not match the size of the
   *    dimensions of the given image data.
   *
   * \param [in] prev_tracks the tracks from previous tracking steps
   * \param [in] frame_number the frame number of the current frame
   * \param [in] image_data the image pixels for the current frame
   * \param [in] mask Optional mask image that uses positive values to denote
   *                  regions of the input image to consider for feature
   *                  tracking. An empty sptr indicates no mask (default
   *                  value).
   * \returns an updated set a tracks including the current frame
   */
  virtual vital::track_set_sptr
  track(vital::track_set_sptr prev_tracks,
        unsigned int frame_number,
        vital::image_container_sptr image_data,
        vital::image_container_sptr mask = vital::image_container_sptr()) const;


private:

  /// The feature detector algorithm to use
  vital::algo::detect_features_sptr detector_;

  /// The descriptor extractor algorithm to use
  vital::algo::extract_descriptors_sptr extractor_;

  /// The feature matching algorithm to use
  vital::algo::match_features_sptr matcher_;

  /// The loop closure algorithm to use
  vital::algo::close_loops_sptr closer_;

  /// The ID to use for the next created track
  mutable unsigned long next_track_id_;

};


} // end namespace core

} // end namespace maptk
} // end namespace kwiver


#endif // MAPTK_PLUGINS_CORE_TRACK_FEATURES_CORE_H_
