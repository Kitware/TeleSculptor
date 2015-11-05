/*ckwg +29
 * Copyright 2014-2015 by Kitware, Inc.
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
 * \brief Header defining the \link maptk::algo::close_loops_bad_frames_only
 *        close_loops_bad_frames_only \endlink algorithm
 */

#ifndef MAPTK_PLUGINS_CORE_CLOSE_LOOPS_BAD_FRAMES_ONLY_H_
#define MAPTK_PLUGINS_CORE_CLOSE_LOOPS_BAD_FRAMES_ONLY_H_

#include <vital/algo/algorithm.h>
#include <vital/types/image_container.h>
#include <vital/types/track_set.h>

#include <vital/algo/match_features.h>
#include <vital/algo/close_loops.h>
#include <vital/config/config_block.h>

#include <maptk/plugins/core/plugin_core_config.h>


namespace kwiver {
namespace maptk {

namespace core
{

/// Attempts to stitch over incomplete or bad input frames.
/**
 * This class attempts to only make short term loop closures
 * due to bad or incomplete feature point tracking. It operates on the
 * principle that when a bad frame occurs, there is generally a lower
 * percentage of feature tracks.
 */
class PLUGIN_CORE_EXPORT close_loops_bad_frames_only
  : public vital::algorithm_impl<close_loops_bad_frames_only, vital::algo::close_loops>
{
public:

  /// Default Constructor
  close_loops_bad_frames_only();

  /// Copy Constructor
  close_loops_bad_frames_only(const close_loops_bad_frames_only&);

  /// Destructor
  virtual ~close_loops_bad_frames_only() MAPTK_DEFAULT_DTOR;

  /// Return the name of this implementation
  virtual std::string impl_name() const { return "bad_frames_only"; }

  /// Returns implementation description
  virtual std::string description() const;

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

  /// Perform basic shot stitching for bad frame detection
  /**
   * \param [in] frame_number the frame number of the current frame
   * \param [in] input the input track set to stitch
   * \param [in] image image data for the current frame
   * \param [in] mask Optional mask image where positive values indicate
   *                  regions to consider in the input image.
   * \returns an updated set a tracks after the stitching operation
   */
  virtual vital::track_set_sptr
  stitch( vital::frame_id_t frame_number,
          vital::track_set_sptr input,
          vital::image_container_sptr image,
          vital::image_container_sptr mask = vital::image_container_sptr() ) const;

protected:

  /// Is bad frame detection enabled?
  bool enabled_;

  /// Stitching percent feature match required
  double percent_match_req_;

  /// Stitching required new valid shot size in frames
  unsigned new_shot_length_;

  /// Max search length for bad frame detection in frames
  unsigned max_search_length_;

  /// The feature matching algorithm to use
  vital::algo::match_features_sptr matcher_;

};


} // end namespace algo

} // end namespace maptk
} // end namespace kwiver


#endif // MAPTK_PLUGINS_CORE_CLOSE_LOOPS_BAD_FRAMES_ONLY_H_
