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
 * \brief compute_ref_homography algorithm definition
 */

#ifndef MAPTK_ALGO_COMPUTE_REF_HOMOGRAPHY_H_
#define MAPTK_ALGO_COMPUTE_REF_HOMOGRAPHY_H_

#include <maptk/core/core_config.h>

#include <vector>

#include <boost/shared_ptr.hpp>

#include <maptk/core/algo/algorithm.h>
#include <maptk/core/track_set.h>
#include <maptk/core/image_container.h>
#include <maptk/core/homography.h>

namespace maptk
{

namespace algo
{


/// Abstract base class for mapping each image to some reference image.
/**
 * This class differs from estimate_homographies in that estimate_homographies
 * simply performs a homography regression from matching feature points. This
 * class is designed to generate different types of homographies from input
 * feature tracks, which can transform each image back to the same coordinate
 * space derived from some initial refrerence image.
 */
class MAPTK_CORE_EXPORT compute_ref_homography
  : public algorithm_def<compute_ref_homography>
{
public:

  /// Return the name of this algorithm
  std::string type_name() const { return "compute_ref_homography"; }

  /// Estimate the transformation which maps some frame to a reference frame
  /**
   * Similarly to track_features, this class was designed to be called in
   * an online fashion for each sequential frame. The output homography
   * will contain a transformation mapping points from the current frame
   * (with frame_id frame_number) to the earliest possible reference frame
   * via post multiplying points on the current frame with the computed
   * homography.
   *
   * \param [in]   frame_number frame identifier for the current frame
   * \param [in]   tracks the set of all tracked features from the image
   * \param return estimated homography
   */
  virtual f2f_homography_sptr
  estimate( frame_id_t frame_number,
            track_set_sptr tracks ) const = 0;

};


/// Shared pointer type of base compute_ref_homography algorithm definition class
typedef boost::shared_ptr<compute_ref_homography> compute_ref_homography_sptr;


} // end namespace algo

} // end namespace maptk


#endif // MAPTK_COMPUTE_REF_HOMOGRAPHY_H_
