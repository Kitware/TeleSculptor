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
 * \brief Header for OCV draw_tracks algorithm
 */

#ifndef MAPTK_PLUGINS_OCV_DRAW_TRACKS_H_
#define MAPTK_PLUGINS_OCV_DRAW_TRACKS_H_

#include <vital/algo/draw_tracks.h>

#include <maptk/plugins/ocv/ocv_config.h>

#include <memory>

namespace kwiver {
namespace maptk {

namespace ocv
{

/// A class for drawing various information about feature tracks
class MAPTK_OCV_EXPORT draw_tracks
: public vital::algorithm_impl<draw_tracks, vital::algo::draw_tracks>
{
public:

  /// Constructor
  draw_tracks();

  /// Copy Constructor
  draw_tracks(const draw_tracks& other);

  /// Destructor
  virtual ~draw_tracks();

  /// Return the name of this implementation
  virtual std::string impl_name() const { return "ocv"; }

  /// Get this algorithm's \link maptk::kwiver::config_block configuration block \endlink
  virtual vital::config_block_sptr get_configuration() const;
  /// Set this algorithm's properties via a config block
  virtual void set_configuration(vital::config_block_sptr config);
  /// Check that the algorithm's currently configuration is valid
  virtual bool check_configuration(vital::config_block_sptr config) const;

  /// Draw features tracks on top of the input images.
  /**
   * This process can either be called in an offline fashion, where all
   * tracks and images are provided to the function on the first call,
   * or in an online fashion where only new images are provided on
   * sequential calls. This function can additionally consumes a second
   * track set for which can optionally be used to display additional
   * information to provide a comparison between the two track sets.
   *
   * \param [in] display_set the main track set to draw
   * \param [in] image_data a list of images the tracks were computed over
   * \param [in] comparison_set optional comparison track set
   * \returns a pointer to the last image generated
   */
  virtual vital::image_container_sptr
  draw(vital::track_set_sptr display_set,
       vital::image_container_sptr_list image_data,
       vital::track_set_sptr comparison_set = vital::track_set_sptr());

private:

  /// private implementation class
  class priv;
  const std::unique_ptr<priv> d_;
};


} // end namespace ocv

} // end namespace maptk
} // end namespace kwiver


#endif // MAPTK_PLUGINS_OCV_DRAW_TRACKS_H_
