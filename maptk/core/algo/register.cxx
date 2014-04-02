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
 * \brief core algorithms registration implementation
 */

#include <maptk/core/algo/register.h>

#include <maptk/core/algo/close_loops_bad_frames_only.h>
#include <maptk/core/algo/close_loops_multi_method.h>
#include <maptk/core/algo/compute_ref_homography_default.h>
#include <maptk/core/algo/convert_image.h>
#include <maptk/core/algo/hierarchical_bundle_adjust.h>
#include <maptk/core/algo/match_features_homography.h>
#include <maptk/core/algo/track_features_default.h>

namespace maptk
{

namespace algo
{

/// register all algorithms in this module
void register_algorithms()
{
  close_loops_bad_frames_only::register_self();
  close_loops_multi_method::register_self();
  compute_ref_homography_default::register_self();
  convert_image_default::register_self();
  hierarchical_bundle_adjust::register_self();
  match_features_homography::register_self();
  track_features_default::register_self();
}


} // end namespace algo

} // end namespace maptk
