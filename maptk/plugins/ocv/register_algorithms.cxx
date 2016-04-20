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
#include <maptk/plugins/ocv/detect_features_AGAST.h>
#include <maptk/plugins/ocv/detect_features_FAST.h>
#include <maptk/plugins/ocv/detect_features_GFTT.h>
#include <maptk/plugins/ocv/detect_features_MSD.h>
#include <maptk/plugins/ocv/detect_features_MSER.h>
#include <maptk/plugins/ocv/detect_features_simple_blob.h>
#include <maptk/plugins/ocv/detect_features_STAR.h>
#include <maptk/plugins/ocv/draw_tracks.h>
#include <maptk/plugins/ocv/estimate_homography.h>
#include <maptk/plugins/ocv/extract_descriptors_BRIEF.h>
#include <maptk/plugins/ocv/extract_descriptors_DAISY.h>
#include <maptk/plugins/ocv/extract_descriptors_FREAK.h>
#include <maptk/plugins/ocv/extract_descriptors_LATCH.h>
#include <maptk/plugins/ocv/extract_descriptors_LUCID.h>
#include <maptk/plugins/ocv/feature_detect_extract_BRISK.h>
#include <maptk/plugins/ocv/feature_detect_extract_ORB.h>
#include <maptk/plugins/ocv/feature_detect_extract_SIFT.h>
#include <maptk/plugins/ocv/feature_detect_extract_SURF.h>
#include <maptk/plugins/ocv/image_io.h>
#include <maptk/plugins/ocv/match_features_bruteforce.h>
#include <maptk/plugins/ocv/match_features_flannbased.h>


namespace kwiver {
namespace maptk {
namespace ocv {


/// Register OCV algorithm implementations with the given or global registrar
int register_algorithms( vital::registrar &reg )
{
  vital::logger_handle_t log = vital::get_logger("ocv_register_algo");

#if defined(HAVE_OPENCV_NONFREE)
  cv::initModule_nonfree();
#endif

  REGISTRATION_INIT( reg );

  REGISTER_TYPE( maptk::ocv::analyze_tracks );
  REGISTER_TYPE( maptk::ocv::draw_tracks );
  REGISTER_TYPE( maptk::ocv::estimate_homography );
  REGISTER_TYPE( maptk::ocv::image_io );

  // OCV Algorithm based class wrappers
  REGISTER_TYPE( maptk::ocv::detect_features_BRISK );
  REGISTER_TYPE( maptk::ocv::detect_features_FAST );
  REGISTER_TYPE( maptk::ocv::detect_features_GFTT );
  REGISTER_TYPE( maptk::ocv::detect_features_MSER );
  REGISTER_TYPE( maptk::ocv::detect_features_ORB );
  REGISTER_TYPE( maptk::ocv::detect_features_simple_blob );

  REGISTER_TYPE( maptk::ocv::extract_descriptors_BRISK );
  REGISTER_TYPE( maptk::ocv::extract_descriptors_ORB );

  REGISTER_TYPE( maptk::ocv::match_features_bruteforce );
  REGISTER_TYPE( maptk::ocv::match_features_flannbased );

  // Conditional algorithms
  // Source ``MAPTK_OCV_HAS_*`` symbol definitions can be found in the header
  //  files of the algorithms referred to.
#ifdef MAPTK_OCV_HAS_AGAST
  REGISTER_TYPE( maptk::ocv::detect_features_AGAST );
#endif

#ifdef MAPTK_OCV_HAS_BRIEF
  REGISTER_TYPE( maptk::ocv::extract_descriptors_BRIEF );
#endif

#ifdef MAPTK_OCV_HAS_DAISY
  REGISTER_TYPE( maptk::ocv::extract_descriptors_DAISY );
#endif

#ifdef MAPTK_OCV_HAS_FREAK
  REGISTER_TYPE( maptk::ocv::extract_descriptors_FREAK );
#endif

#ifdef MAPTK_OCV_HAS_LATCH
  REGISTER_TYPE( maptk::ocv::extract_descriptors_LATCH );
#endif

#ifdef MAPTK_OCV_HAS_LUCID
  REGISTER_TYPE( maptk::ocv::extract_descriptors_LUCID );
#endif

#ifdef MAPTK_OCV_HAS_MSD
  REGISTER_TYPE( maptk::ocv::detect_features_MSD );
#endif

#ifdef MAPTK_OCV_HAS_SIFT
  REGISTER_TYPE( maptk::ocv::detect_features_SIFT );
  REGISTER_TYPE( maptk::ocv::extract_descriptors_SIFT );
#endif

#ifdef MAPTK_OCV_HAS_STAR
  REGISTER_TYPE( maptk::ocv::detect_features_STAR );
#endif

#ifdef MAPTK_OCV_HAS_SURF
  REGISTER_TYPE( maptk::ocv::detect_features_SURF );
  REGISTER_TYPE( maptk::ocv::extract_descriptors_SURF );
#endif

  REGISTRATION_SUMMARY();
  return REGISTRATION_FAILURES();
}


#undef REGISTER_TYPE_OCV2
#undef REGISTER_TYPE_OCV3


} // end namespace ocv
} // end namespace maptk
} // end namespace kwiver
