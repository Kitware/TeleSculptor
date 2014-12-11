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
 * \brief Register VXL algorithms implementation
 */

#include "register_algorithms.h"

#include <maptk/plugins/vxl/bundle_adjust.h>
#include <maptk/plugins/vxl/close_loops_homography_guided.h>
#include <maptk/plugins/vxl/estimate_essential_matrix.h>
#include <maptk/plugins/vxl/estimate_homography.h>
#include <maptk/plugins/vxl/estimate_similarity_transform.h>
#include <maptk/plugins/vxl/image_io.h>
#include <maptk/plugins/vxl/initialize_cameras_landmarks.h>
#include <maptk/plugins/vxl/optimize_cameras.h>
#include <maptk/plugins/vxl/triangulate_landmarks.h>


namespace maptk
{

namespace vxl
{

/// Register VXL algorithm implementations with the given or global registrar
int register_algorithms( maptk::registrar &reg )
{
  try
  {
    LOG_DEBUG( "plugin::vxl::register_algo_impls",
               "Registering VXL algo implementations" );

    int expected = 9,
        registered
      = maptk::vxl::bundle_adjust::register_self( reg )
      + maptk::vxl::close_loops_homography_guided::register_self( reg )
      + maptk::vxl::estimate_essential_matrix::register_self( reg )
      + maptk::vxl::estimate_homography::register_self( reg )
      + maptk::vxl::estimate_similarity_transform::register_self( reg )
      + maptk::vxl::image_io::register_self( reg )
      + maptk::vxl::initialize_cameras_landmarks::register_self( reg )
      + maptk::vxl::optimize_cameras::register_self( reg )
      + maptk::vxl::triangulate_landmarks::register_self( reg )
      ;

    LOG_DEBUG( "plugin::vxl::register_algo_impls",
               "Registered " << registered << " of " << expected << " algorithms" );

    return expected - registered;
  }
  catch (...)
  {
    LOG_ERROR( "plugin::vxl::register_algo_impls",
               "Exception caught during algorithm registration" );
  }
  return -1;
}

} // end vxl ns

} // end maptk ns
