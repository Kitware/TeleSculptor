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
 * \brief OpenCV algorithm registration implementation
 */

#include "register_algorithms.h"

#include <opencv2/opencv_modules.hpp>
#ifdef HAVE_OPENCV_NONFREE
#include <opencv2/nonfree/nonfree.hpp>
#endif

#include <maptk/plugin_interface/algorithm_plugin_interface_macros.h>
#include <maptk/plugins/ocv/analyze_tracks.h>
#include <maptk/plugins/ocv/detect_features.h>
#include <maptk/plugins/ocv/draw_tracks.h>
#include <maptk/plugins/ocv/estimate_homography.h>
#include <maptk/plugins/ocv/extract_descriptors.h>
#include <maptk/plugins/ocv/image_io.h>
#include <maptk/plugins/ocv/match_features.h>


namespace kwiver {
namespace maptk {
namespace ocv {

/// Register OCV algorithm implementations with the given or global registrar
int register_algorithms( vital::registrar &reg )
{
#ifdef HAVE_OPENCV_NONFREE
  cv::initModule_nonfree();
#endif

  REGISTRATION_INIT( reg );

  REGISTER_TYPE( maptk::ocv::analyze_tracks );
  REGISTER_TYPE( maptk::ocv::detect_features );
  REGISTER_TYPE( maptk::ocv::draw_tracks );
  REGISTER_TYPE( maptk::ocv::estimate_homography );
  REGISTER_TYPE( maptk::ocv::extract_descriptors );
  REGISTER_TYPE( maptk::ocv::image_io );
  REGISTER_TYPE( maptk::ocv::match_features );

  REGISTRATION_SUMMARY();
  return REGISTRATION_FAILURES();
}

} // end namespace ocv
} // end namespace maptk
} // end namespace kwiver
