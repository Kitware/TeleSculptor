/*ckwg +29
 * Copyright 2015-2016 by Kitware, Inc.
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

#ifndef MAPTK_PLUGINS_VXL_ESTIMATE_CANONICAL_TRANSFORM_H_
#define MAPTK_PLUGINS_VXL_ESTIMATE_CANONICAL_TRANSFORM_H_

#include <vital/vital_config.h>
#include <maptk/plugins/vxl/maptk_vxl_export.h>

#include <vital/algo/estimate_canonical_transform.h>

#include <memory>

/**
 * \file
 * \brief Header defining the VXL estimate_canonical_transform algorithm
 */

namespace kwiver {
namespace maptk {
namespace vxl {


/// Algorithm for estimating a canonical transform for cameras and landmarks
/**
 *  A canonical transform is a repeatable transformation that can be recovered
 *  from data.  In this case we assume at most a similarity transformation.
 *  If data sets P1 and P2 are equivalent up to a similarity transformation,
 *  then applying a canonical transform to P1 and separately a
 *  canonical transform to P2 should bring the data into the same coordinates.
 *
 *  This implementation first fits a "ground" plane to the landmark points
 *  using robust estimation methods provided by the rrel library in VXL.
 *  It then estimates the remaining degrees of freedom using PCA much like
 *  the implementation in the core plugin.  The scale is set to normalize the
 *  landmarks to unit standard deviation.
 */
class MAPTK_VXL_EXPORT estimate_canonical_transform
  : public vital::algorithm_impl<estimate_canonical_transform,
                                 vital::algo::estimate_canonical_transform>
{
public:
  /// Constructor
  estimate_canonical_transform();

  /// Destructor
  virtual ~estimate_canonical_transform();

  /// Copy Constructor
  estimate_canonical_transform(const estimate_canonical_transform& other);

  /// Return the name of this implementation
  virtual std::string impl_name() const { return "vxl_plane"; }

  /// Get this algorithm's \link vital::config_block configuration block \endlink
  virtual vital::config_block_sptr get_configuration() const;
  /// Set this algorithm's properties via a config block
  virtual void set_configuration(vital::config_block_sptr config);
  /// Check that the algorithm's configuration config_block is valid
  virtual bool check_configuration(vital::config_block_sptr config) const;

  /// Estimate a canonical similarity transform for cameras and points
  /**
   * \param cameras The camera map containing all the cameras
   * \param landmarks The landmark map containing all the 3D landmarks
   * \throws algorithm_exception When the data is insufficient or degenerate.
   * \returns An estimated similarity transform mapping the data to the
   *          canonical space.
   * \note This algorithm does not apply the transformation, it only estimates it.
   */
  virtual kwiver::vital::similarity_d
  estimate_transform(kwiver::vital::camera_map_sptr const cameras,
                     kwiver::vital::landmark_map_sptr const landmarks) const;

private:
  /// private implementation class
  class priv;
  const std::unique_ptr<priv> d_;
};

} // end namespace vxl
} // end namespace maptk
} // end namespace kwiver


#endif // MAPTK_PLUGINS_VXL_ESTIMATE_CANONICAL_TRANSFORM_H_
