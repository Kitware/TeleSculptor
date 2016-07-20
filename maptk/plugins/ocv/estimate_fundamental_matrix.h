/*ckwg +29
 * Copyright 2016 by Kitware, Inc.
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
 * \brief OCV estimate_fundamental_matrix algorithm impl interface
 */

#ifndef MAPTK_PLUGINS_OCV_ESTIMATE_FUNDAMENTAL_MATRIX_H_
#define MAPTK_PLUGINS_OCV_ESTIMATE_FUNDAMENTAL_MATRIX_H_


#include <vital/vital_config.h>
#include <maptk/plugins/ocv/maptk_ocv_export.h>

#include <vital/algo/estimate_fundamental_matrix.h>


namespace kwiver {
namespace maptk {

namespace ocv
{

/// A class that using OpenCV to estimate a fundamental matrix from matching 2D points
class MAPTK_OCV_EXPORT estimate_fundamental_matrix
  : public vital::algorithm_impl<estimate_fundamental_matrix, vital::algo::estimate_fundamental_matrix>
{
public:
    /// Constructor
  estimate_fundamental_matrix();

  /// Destructor
  virtual ~estimate_fundamental_matrix();

  /// Copy Constructor
  estimate_fundamental_matrix(const estimate_fundamental_matrix& other);
  
  /// Return the name of this implementation
  virtual std::string impl_name() const { return "ocv"; }

  /// Get this algorithm's \link vital::config_block configuration block \endlink
  virtual vital::config_block_sptr get_configuration() const;

  /// Set this algorithm's properties via a config block
  virtual void set_configuration(vital::config_block_sptr config);

  /// Check that the algorithm's configuration config_block is valid
  virtual bool check_configuration(vital::config_block_sptr config) const;

  /// Estimate a fundamental matrix from corresponding points
  /**
   * If estimation fails, a NULL-containing sptr is returned
   *
   * \param [in]  pts1 the vector or corresponding points from the source image
   * \param [in]  pts2 the vector of corresponding points from the destination image
   * \param [out] inliers for each point pair, the value is true if
   *                      this pair is an inlier to the fundamental matrix estimate
   * \param [in]  inlier_scale error distance tolerated for matches to be inliers
   */
  virtual vital::fundamental_matrix_sptr
  estimate(const std::vector<vital::vector_2d>& pts1,
           const std::vector<vital::vector_2d>& pts2,
           std::vector<bool>& inliers,
           double inlier_scale = 3.0) const;
  using vital::algo::estimate_fundamental_matrix::estimate;

private:
  /// private implementation class
  class priv;
  const std::unique_ptr<priv> d_;
};


} // end namespace ocv

} // end namespace maptk
} // end namespace kwiver


#endif // MAPTK_PLUGINS_OCV_ESTIMATE_FUNDAMENTAL_MATRIX_H_
