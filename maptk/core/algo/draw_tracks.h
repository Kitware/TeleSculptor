/*ckwg +29
 * Copyright 2014 by Kitware, Inc.
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

#ifndef MAPTK_ALGO_DRAW_TRACKS_H_
#define MAPTK_ALGO_DRAW_TRACKS_H_

#include <maptk/core/core_config.h>

#include <boost/shared_ptr.hpp>

#include <maptk/core/algo/algorithm.h>
#include <maptk/core/image_container.h>
#include <maptk/core/track_set.h>

#include <ostream>

/**
 * \file
 * \brief Header defining an abstract \link maptk::algo::draw_tracks track
 *        drawing \endlink algorithm
 */

namespace maptk
{

namespace algo
{


/// An abstract base class for algorithms which draw tracks on top of
/// images in various ways, for analyzing results.
class MAPTK_CORE_EXPORT draw_tracks
  : public algorithm_def<draw_tracks>
{
public:

  /// Return the name of this algorithm.
  std::string type_name() const { return "draw_tracks"; }

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
   * \param returns a pointer to the last image generated
   */
  virtual image_container_sptr
  draw(track_set_sptr display_set,
       image_container_sptr_list image_data,
       track_set_sptr comparison_set = track_set_sptr()) = 0;

};


/// A smart pointer to a draw_tracks instance.
typedef boost::shared_ptr<draw_tracks> draw_tracks_sptr;


} // end namespace algo

} // end namespace maptk


#endif // MAPTK_ALGO_DRAW_TRACKS_H_
