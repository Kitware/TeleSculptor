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
 * \brief VXL essential matrix estimation algorithm (5 point alg)
 */

#ifndef MAPTK_PLUGINS_VXL_ESTIMATE_ESSENTIAL_MATRIX_H_
#define MAPTK_PLUGINS_VXL_ESTIMATE_ESSENTIAL_MATRIX_H_

#include <boost/scoped_ptr.hpp>

#include <vital/types/camera_intrinsics.h>

#include <vital/algo/estimate_essential_matrix.h>
#include <maptk/plugins/vxl/vxl_config.h>


namespace kwiver {
namespace maptk {

namespace vxl
{

/// A class that uses 5 pt algorithm to estimate an initial xform between 2 pt sets
class MAPTK_VXL_EXPORT estimate_essential_matrix
  : public kwiver::vital::algorithm_impl<estimate_essential_matrix, kwiver::vital::algo::estimate_essential_matrix>
{
public:
  /// Constructor
  estimate_essential_matrix();

  /// Destructor
  virtual ~estimate_essential_matrix();

  /// Copy Constructor
  estimate_essential_matrix(const estimate_essential_matrix& other);

  /// Return the name of this implementation
  std::string impl_name() const { return "vxl"; }

  /// Get this algorithm's \link kwiver::vital::config_block configuration block \endlink
  virtual kwiver::vital::config_block_sptr get_configuration() const;
  /// Set this algorithm's properties via a config block
  virtual void set_configuration(kwiver::vital::config_block_sptr config);
  /// Check that the algorithm's currently configuration is valid
  virtual bool check_configuration(kwiver::vital::config_block_sptr config) const;

  /// Estimate an essential matrix from corresponding points
  /**
   * \param [in]  pts1 the vector or corresponding points from the first image
   * \param [in]  pts2 the vector of corresponding points from the second image
   * \param [in]  cal1 the intrinsic parameters of the first camera
   * \param [in]  cal2 the intrinsic parameters of the second camera
   * \param [out] inliers for each point pa:wir, the value is true if
   *                      this pair is an inlier to the estimate
   * \param [in]  inlier_scale error distance tolerated for matches to be inliers
   */
  virtual
  kwiver::vital::essential_matrix_sptr
  estimate(const std::vector<kwiver::vital::vector_2d>& pts1,
           const std::vector<kwiver::vital::vector_2d>& pts2,
           const kwiver::vital::camera_intrinsics_d &cal1,
           const kwiver::vital::camera_intrinsics_d &cal2,
           std::vector<bool>& inliers,
           double inlier_scale = 1.0) const;

private:
  /// private implementation class
  class priv;
  boost::scoped_ptr<priv> d_;
};


} // end namespace vxl

} // end namespace maptk
} // end namespace kwiver


#endif // MAPTK_PLUGINS_VXL_ESTIMATE_ESSENTIAL_MATRIX_H_
