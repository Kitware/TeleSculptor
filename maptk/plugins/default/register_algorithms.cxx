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
 * \brief Defaults plugin algorithm registration interface impl
 */

#include "register_algorithms.h"

#include <iostream>

#include <maptk/logging_macros.h>
#include <maptk/plugins/default/close_loops_bad_frames_only.h>
#include <maptk/plugins/default/close_loops_multi_method.h>
#include <maptk/plugins/default/compute_ref_homography_default.h>
#include <maptk/plugins/default/convert_image_default.h>
#include <maptk/plugins/default/hierarchical_bundle_adjust.h>
#include <maptk/plugins/default/match_features_homography.h>
#include <maptk/plugins/default/plugin_default_config.h>
#include <maptk/plugins/default/track_features_default.h>


namespace maptk
{

namespace defaults
{

// Register default algorithms with the given or global registrar
int register_algo_impls(maptk::registrar &reg)
{
  try
  {
    LOG_DEBUG( "plugin::default::register_algo_impls",
               "Registering DEFAULT algo implementations (" << &reg << ")" );

    int expected = 7,
        registered
      = maptk::algo::close_loops_bad_frames_only::register_self(reg)
      + maptk::algo::close_loops_multi_method::register_self(reg)
      + maptk::algo::compute_ref_homography_default::register_self(reg)
      + maptk::algo::convert_image_default::register_self(reg)
      + maptk::algo::hierarchical_bundle_adjust::register_self(reg)
      + maptk::algo::match_features_homography::register_self(reg)
      + maptk::algo::track_features_default::register_self(reg)
      ;

    LOG_DEBUG( "plugin::default::register_algo_impls",
               "Registered algorithms. Succeeded: " << registered << " of " << expected);
    return expected - registered;
  }
  catch (...)
  {
    LOG_ERROR( "plugin::default::register_algo_impls",
               "Exception caught during algorithm registrarion" );
  }
  return -1;
}

} // end defaults namespace

} // end maptk namespace
