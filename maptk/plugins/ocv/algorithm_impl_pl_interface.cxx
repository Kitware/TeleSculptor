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
 * \brief OpenCV plugin algorithm registration plugin interface impl
 */

#include <maptk/logging_macros.h>
#include <maptk/plugin_interface/algorithm_impl_pl_interface.h>

#include <opencv2/opencv_modules.hpp>
#ifdef HAVE_OPENCV_NONFREE
#include <opencv2/nonfree/nonfree.hpp>
#endif

#include <maptk/plugins/ocv/analyze_tracks.h>
#include <maptk/plugins/ocv/detect_features.h>
#include <maptk/plugins/ocv/draw_tracks.h>
#include <maptk/plugins/ocv/estimate_homography.h>
#include <maptk/plugins/ocv/extract_descriptors.h>
#include <maptk/plugins/ocv/image_io.h>
#include <maptk/plugins/ocv/match_features.h>
#include <maptk/plugins/ocv/ocv_config.h>


#ifdef __cplusplus
extern "C"
{
#endif


MAPTK_OCV_EXPORT
int register_algo_impls(maptk::registrar &reg)
{
  LOG_DEBUG("plugin::ocv::register_algo_impls",
            "Registering OCV plugin algo implementations");

#ifdef HAVE_OPENCV_NONFREE
  cv::initModule_nonfree();
#endif

  int registered
    = maptk::ocv::analyze_tracks::register_self(reg)
    + maptk::ocv::detect_features::register_self(reg)
    + maptk::ocv::draw_tracks::register_self(reg)
    + maptk::ocv::estimate_homography::register_self(reg)
    + maptk::ocv::extract_descriptors::register_self(reg)
    + maptk::ocv::image_io::register_self(reg)
    + maptk::ocv::match_features::register_self(reg)
    ;

  LOG_DEBUG("plugin::ocv::register_algo_impls",
            "Registered algorithms. Returned: " << registered);
  return 7 - registered;
}


#ifdef __cplusplus
}
#endif
