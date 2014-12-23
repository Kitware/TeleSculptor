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
#include <maptk/plugin_interface/algorithm_plugin_interface_macros.h>
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
int register_algorithms(maptk::registrar &reg)
{
    LOG_DEBUG( "maptk::plugins::default::register_algorithms",
               "Registering DEFAULT algo implementations (" << &reg << ")" );

    REGISTRATION_INIT( reg );

    REGISTER_TYPE( maptk::algo::close_loops_bad_frames_only );
    REGISTER_TYPE( maptk::algo::close_loops_multi_method );
    REGISTER_TYPE( maptk::algo::compute_ref_homography_default );
    REGISTER_TYPE( maptk::algo::convert_image_default );
    REGISTER_TYPE( maptk::algo::hierarchical_bundle_adjust );
    REGISTER_TYPE( maptk::algo::match_features_homography );
    REGISTER_TYPE( maptk::algo::track_features_default );

    REGISTRATION_SUMMARY();
    return REGISTRATION_FAILURES();
}

} // end defaults namespace

} // end maptk namespace
