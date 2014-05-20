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

/**
 * \file
 * \brief Header defining abstract \link maptk::algo::triangulate_landmarks
 *        triangulate landmarks \endlink algorithm
 */

#ifndef MAPTK_ALGO_TRIANGULATE_LANDMARKS_H_
#define MAPTK_ALGO_TRIANGULATE_LANDMARKS_H_

#include <maptk/core/algo/algorithm.h>
#include <maptk/core/track_set.h>
#include <maptk/core/camera_map.h>
#include <maptk/core/landmark_map.h>


namespace maptk
{

namespace algo
{

/// An abstract base class for triangulating landmarks
class MAPTK_CORE_EXPORT triangulate_landmarks
: public algorithm_def<triangulate_landmarks>
{
public:
  /// Return the name of this algorithm
  virtual std::string type_name() const { return "triangulate_landmarks"; }

  /// Triangulate the landmark locations given sets of cameras and tracks
  /**
   * \param [in] cameras the cameras viewing the landmarks
   * \param [in] tracks the tracks to use as constraints
   * \param [in,out] landmarks the landmarks to triangulate
   *
   * This function only triangulates the landmarks with indicies in the
   * landmark map and which have support in the tracks and cameras
   */
  virtual void
  triangulate(camera_map_sptr cameras,
              track_set_sptr tracks,
              landmark_map_sptr& landmarks) const = 0;
};


/// type definition for shared pointer to a triangulate landmarks algorithm
typedef boost::shared_ptr<triangulate_landmarks> triangulate_landmarks_sptr;


} // end namespace algo

} // end namespace maptk


#endif // MAPTK_ALGO_TRIANGULATE_LANDMARKS_H_
