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
* \brief Header defining VXL algorithm implementation of camera optimization.
*/

#ifndef MAPTK_PLUGINS_VXL_OPTIMIZE_CAMERAS_H_
#define MAPTK_PLUGINS_VXL_OPTIMIZE_CAMERAS_H_

#include <string>

#include <vital/algo/algorithm.h>
#include <maptk/algo/optimize_cameras.h>
#include <maptk/plugins/vxl/vxl_config.h>


namespace maptk
{

namespace vxl
{


class MAPTK_VXL_EXPORT optimize_cameras
  : public kwiver::vital::algorithm_impl<optimize_cameras, algo::optimize_cameras>
{
public:
  virtual std::string impl_name() const { return "vxl"; }

  /// \cond DoxygenSuppress
  virtual void set_configuration(kwiver::vital::config_block_sptr /*config*/) { }
  virtual bool check_configuration(kwiver::vital::config_block_sptr /*config*/) const { return true; }
  /// \endcond

  /// Optimize camera parameters given sets of landmarks and tracks
  /**
   * We only optimize cameras that have associating tracks and landmarks in
   * the given maps.
   *
   * \throws invalid_value When one or more of the given pointer is Null.
   *
   * \param[in,out] cameras   Cameras to optimize.
   * \param[in]     tracks    The tracks to use as constraints.
   * \param[in]     landmarks The landmarks the cameras are viewing.
   */
  virtual void
  optimize(kwiver::vital::camera_map_sptr & cameras,
           kwiver::vital::track_set_sptr tracks,
           kwiver::vital::landmark_map_sptr landmarks) const;
};


}

}

#endif // MAPTK_PLUGINS_VXL_OPTIMIZE_CAMERAS_H_
