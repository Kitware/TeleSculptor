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
 * \brief Header for VXL bundle adjustment algorithm
 */

#ifndef MAPTK_PLUGINS_VXL_BUNDLE_ADJUST_H_
#define MAPTK_PLUGINS_VXL_BUNDLE_ADJUST_H_

#include <boost/scoped_ptr.hpp>

#include <vital/algo/bundle_adjust.h>
#include <maptk/plugins/vxl/vxl_config.h>


namespace kwiver {
namespace maptk {

namespace vxl
{

/// A class for bundle adjustment of tracks using VXL
class MAPTK_VXL_EXPORT bundle_adjust
: public vital::algorithm_impl<bundle_adjust, vital::algo::bundle_adjust>
{
public:
  /// Constructor
  bundle_adjust();

  /// Destructor
  virtual ~bundle_adjust();

  /// Copy Constructor
  bundle_adjust(const bundle_adjust& other);

  /// Return the name of this implementation
  virtual std::string impl_name() const { return "vxl"; }

  /// Get this algorithm's \link maptk::vital::config_block configuration block \endlink
  virtual vital::config_block_sptr get_configuration() const;
  /// Set this algorithm's properties via a config block
  virtual void set_configuration(vital::config_block_sptr config);
  /// Check that the algorithm's currently configuration is valid
  virtual bool check_configuration(vital::config_block_sptr config) const;

  /// Optimize the camera and landmark parameters given a set of tracks
  /**
   * \param [in,out] cameras the cameras to optimize
   * \param [in,out] landmarks the landmarks to optimize
   * \param [in] tracks the tracks to use as constraints
   */
  virtual void
  optimize(vital::camera_map_sptr& cameras,
           vital::landmark_map_sptr& landmarks,
           vital::track_set_sptr tracks) const;

private:
  /// private implementation class
  class priv;
  boost::scoped_ptr<priv> d_;
};


} // end namespace vxl

} // end namespace maptk
} // end namespace kwiver


#endif // MAPTK_PLUGINS_VXL_BUNDLE_ADJUST_H_
