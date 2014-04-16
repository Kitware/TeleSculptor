/*ckwg +29
 * Copyright 2013-2014 by Kitware, Inc.
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
 * \brief VXL algorithm registration implementation
 */

#include <maptk/vxl/register.h>

#include <maptk/vxl/bundle_adjust.h>
#include <maptk/vxl/close_loops_homography_guided.h>
#include <maptk/vxl/estimate_essential_matrix.h>
#include <maptk/vxl/estimate_homography.h>
#include <maptk/vxl/estimate_similarity_transform.h>
#include <maptk/vxl/image_io.h>
#include <maptk/vxl/optimize_cameras.h>
#include <maptk/vxl/triangulate_landmarks.h>

namespace maptk
{

namespace vxl
{

/// register all algorithms in this module
void register_algorithms()
{
  vxl::bundle_adjust::register_self();
  vxl::close_loops_homography_guided::register_self();
  vxl::estimate_essential_matrix::register_self();
  vxl::estimate_homography::register_self();
  vxl::estimate_similarity_transform::register_self();
  vxl::image_io::register_self();
  vxl::optimize_cameras::register_self();
  vxl::triangulate_landmarks::register_self();
}


} // end namespace vxl

} // end namespace maptk
