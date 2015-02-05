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
 * \brief Register VXL algorithms implementation
 */

#include "register_algorithms.h"

#include <maptk/logging_macros.h>
#include <maptk/plugin_interface/algorithm_plugin_interface_macros.h>
#include <maptk/plugins/vxl/bundle_adjust.h>
#include <maptk/plugins/vxl/close_loops_homography_guided.h>
#include <maptk/plugins/vxl/estimate_essential_matrix.h>
#include <maptk/plugins/vxl/estimate_homography.h>
#include <maptk/plugins/vxl/estimate_similarity_transform.h>
#include <maptk/plugins/vxl/image_io.h>
#include <maptk/plugins/vxl/initialize_cameras_landmarks.h>
#include <maptk/plugins/vxl/optimize_cameras.h>
#include <maptk/plugins/vxl/triangulate_landmarks.h>
#include <maptk/plugins/vxl/match_features_constrained.h>


namespace maptk
{

namespace vxl
{

/// Register VXL algorithm implementations with the given or global registrar
int register_algorithms( maptk::registrar &reg )
{
  REGISTRATION_INIT( reg );

  REGISTER_TYPE( maptk::vxl::bundle_adjust );
  REGISTER_TYPE( maptk::vxl::close_loops_homography_guided );
  REGISTER_TYPE( maptk::vxl::estimate_essential_matrix );
  REGISTER_TYPE( maptk::vxl::estimate_homography );
  REGISTER_TYPE( maptk::vxl::estimate_similarity_transform );
  REGISTER_TYPE( maptk::vxl::image_io );
  REGISTER_TYPE( maptk::vxl::initialize_cameras_landmarks );
  REGISTER_TYPE( maptk::vxl::optimize_cameras );
  REGISTER_TYPE( maptk::vxl::triangulate_landmarks );
  REGISTER_TYPE( maptk::vxl::match_features_constrained );

  REGISTRATION_SUMMARY();
  return REGISTRATION_FAILURES();
}

} // end vxl ns

} // end maptk ns
