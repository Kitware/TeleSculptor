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
 * \brief Defaults plugin algorithm registration interface impl
 */

#include "register_algorithms.h"

#include <maptk/plugin_interface/algorithm_plugin_interface_macros.h>
#include <maptk/plugins/core/close_loops_bad_frames_only.h>
#include <maptk/plugins/core/close_loops_multi_method.h>
#include <maptk/plugins/core/compute_ref_homography_core.h>
#include <maptk/plugins/core/convert_image_bypass.h>
#include <maptk/plugins/core/filter_features_magnitude.h>
#include <maptk/plugins/core/hierarchical_bundle_adjust.h>
#include <maptk/plugins/core/initialize_cameras_landmarks.h>
#include <maptk/plugins/core/match_features_homography.h>
#include <maptk/plugins/core/plugin_core_config.h>
#include <maptk/plugins/core/track_features_core.h>
#include <maptk/plugins/core/triangulate_landmarks.h>


namespace maptk
{

namespace core
{

// Register core algorithms with the given or global registrar
int register_algorithms(kwiver::vital::registrar &reg)
{
    REGISTRATION_INIT( reg );

    REGISTER_TYPE( maptk::core::close_loops_bad_frames_only );
    REGISTER_TYPE( maptk::core::close_loops_multi_method );
    REGISTER_TYPE( maptk::core::compute_ref_homography_core );
    REGISTER_TYPE( maptk::core::convert_image_bypass );
    REGISTER_TYPE( maptk::core::filter_features_magnitude );
    REGISTER_TYPE( maptk::core::hierarchical_bundle_adjust );
    REGISTER_TYPE( maptk::core::initialize_cameras_landmarks );
    REGISTER_TYPE( maptk::core::match_features_homography );
    REGISTER_TYPE( maptk::core::track_features_core );
    REGISTER_TYPE( maptk::core::triangulate_landmarks );

    REGISTRATION_SUMMARY();
    return REGISTRATION_FAILURES();
}

} // end core namespace

} // end maptk namespace
