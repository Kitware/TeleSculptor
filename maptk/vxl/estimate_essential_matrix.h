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
 * \brief VXL essential matrix estimation algorithm (5 point alg)
 */

#ifndef MAPTK_VXL_ESTIMATE_ESSENTIAL_MATRIX_H_
#define MAPTK_VXL_ESTIMATE_ESSENTIAL_MATRIX_H_

#include "vxl_config.h"

#include <maptk/core/algo/estimate_essential_matrix.h>
#include <maptk/core/camera_intrinsics.h>

namespace maptk
{

namespace vxl
{

/// A class that uses 5 pt algorithm to estimate an initial xform between 2 pt sets
class MAPTK_VXL_EXPORT estimate_essential_matrix
  : public algo::algorithm_impl<estimate_essential_matrix, algo::estimate_essential_matrix>
{
public:
  /// Return the name of this implementation
  std::string impl_name() const { return "vxl"; }

  // No configuration yet for this class.
  /// \cond DoxygenSuppress
  virtual void set_configuration(config_block_sptr /*config*/) {}
  virtual bool check_configuration(config_block_sptr /*config*/) const { return true; }
  /// \endcond

  /// Estimate an essential matrix from corresponding features
  /**
   * Assumes both images are using the same calibration matrix (i.e. focal length)
   * \param [in]  feat1 the set of all features from the first image
   * \param [in]  feat2 the set of all features from the second image
   * \param [in]  matches the set of correspondences between \a feat1 and \a feat2
   * \param [in]  focal length of camera
   */
  matrix_3x3d
  estimate(feature_set_sptr feat1,
           feature_set_sptr feat2,
           match_set_sptr matches,
           const camera_intrinsics_d &cal) const;

};


} // end namespace vxl

} // end namespace maptk


#endif // MAPTK_VXL_ESTIMATE_ESSENTIAL_MATRIX_H_
