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
 * \brief Header defining abstract \link maptk::algo::bundle_adjust
 *        bundle adjustment \endlink algorithm
 */

#ifndef MAPTK_ALGO_BUNDLE_ADJUST_H_
#define MAPTK_ALGO_BUNDLE_ADJUST_H_

#include <maptk/config.h>

#include <vital/algorithm.h>
#include <vital/track_set.h>
#include <vital/camera_map.h>
#include <vital/landmark_map.h>


namespace maptk
{

namespace algo
{

/// An abstract base class for bundle adjustment using tracks
class MAPTK_LIB_EXPORT bundle_adjust
  : public kwiver::vital::algorithm_def<bundle_adjust>
{
public:
  /// Return the name of this algorithm
  static std::string static_type_name() { return "bundle_adjust"; }

  /// Optimize the camera and landmark parameters given a set of tracks
  /**
   * \param [in,out] cameras the cameras to optimize
   * \param [in,out] landmarks the landmarks to optimize
   * \param [in] tracks the tracks to use as constraints
   */
  virtual void
  optimize(kwiver::vital::camera_map_sptr& cameras,
           kwiver::vital::landmark_map_sptr& landmarks,
           kwiver::vital::track_set_sptr tracks) const = 0;
};


/// type definition for shared pointer to a bundle adjust algorithm
typedef boost::shared_ptr<bundle_adjust> bundle_adjust_sptr;


} // end namespace algo

} // end namespace maptk


#endif // MAPTK_ALGO_BUNDLE_ADJUST_H_
