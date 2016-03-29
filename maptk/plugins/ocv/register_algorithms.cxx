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
#include <maptk/plugins/ocv/detect_features_MSER.h>
#include <maptk/plugins/ocv/detect_features_simple_blob.h>
#include <maptk/plugins/ocv/draw_tracks.h>
#include <maptk/plugins/ocv/estimate_homography.h>
#include <maptk/plugins/ocv/extract_descriptors_BRIEF.h>
#include <maptk/plugins/ocv/extract_descriptors_FREAK.h>
#include <maptk/plugins/ocv/feature_detect_extract_BRISK.h>
#include <maptk/plugins/ocv/feature_detect_extract_ORB.h>
#include <maptk/plugins/ocv/feature_detect_extract_SIFT.h>
#include <maptk/plugins/ocv/image_io.h>
#include <maptk/plugins/ocv/match_features_bruteforce.h>
#include <maptk/plugins/ocv/match_features_flannbased.h>


namespace kwiver {
namespace maptk {
namespace ocv {


// Macro for registering, or not, non-free types based on what OpenCV has
#if defined(HAVE_OPENCV_NONFREE) || defined(HAVE_OPENCV_XFEATURES2D)
  #define REGISTER_TYPE_NF( T ) REGISTER_TYPE( T )
#else
  #define REGISTER_TYPE_NF( T )
#endif //nonfree/xfeatures2d check

#ifndef MAPTK_HAS_OPENCV_VER_3
  #define REGISTER_TYPE_OCV2( T ) REGISTER_TYPE( T );
  #define REGISTER_TYPE_OCV2_NF( T ) REGISTER_TYPE_NF( T );
  #define REGISTER_TYPE_OCV3( T )
  #define REGISTER_TYPE_OCV3_NF( T )
#else
  #define REGISTER_TYPE_OCV2( T )
  #define REGISTER_TYPE_OCV2_NF( T )
  #define REGISTER_TYPE_OCV3( T ) REGISTER_TYPE( T );
  #define REGISTER_TYPE_OCV3_NF( T ) REGISTER_TYPE_NF( T );
#endif //MAPTK_HAS_OPENCV_VER_3


/// Register OCV algorithm implementations with the given or global registrar
int register_algorithms( vital::registrar &reg )
{
#if defined(HAVE_OPENCV_NONFREE)
  cv::initModule_nonfree();
#endif

  REGISTRATION_INIT( reg );

  REGISTER_TYPE( maptk::ocv::analyze_tracks );
  REGISTER_TYPE( maptk::ocv::draw_tracks );
  REGISTER_TYPE( maptk::ocv::estimate_homography );
  REGISTER_TYPE( maptk::ocv::image_io );

  // OCV Algorithm based class wrappers
  REGISTER_TYPE_OCV3( maptk::ocv::detect_features_AGAST );
  REGISTER_TYPE     ( maptk::ocv::detect_features_BRISK );
  REGISTER_TYPE     ( maptk::ocv::detect_features_FAST );
  REGISTER_TYPE     ( maptk::ocv::detect_features_GFTT );
  REGISTER_TYPE     ( maptk::ocv::detect_features_MSER );
  REGISTER_TYPE     ( maptk::ocv::detect_features_ORB );
  REGISTER_TYPE_NF  ( maptk::ocv::detect_features_SIFT );
  REGISTER_TYPE     ( maptk::ocv::detect_features_simple_blob );

  REGISTER_TYPE_OCV2( maptk::ocv::extract_descriptors_BRIEF );
  REGISTER_TYPE     ( maptk::ocv::extract_descriptors_BRISK );
  REGISTER_TYPE_OCV2( maptk::ocv::extract_descriptors_FREAK );
  REGISTER_TYPE     ( maptk::ocv::extract_descriptors_ORB );
  REGISTER_TYPE_NF  ( maptk::ocv::extract_descriptors_SIFT );

  REGISTER_TYPE     ( maptk::ocv::match_features_bruteforce );
  REGISTER_TYPE     ( maptk::ocv::match_features_flannbased );

  REGISTRATION_SUMMARY();
  return REGISTRATION_FAILURES();
}


#undef REGISTER_TYPE_OCV2
#undef REGISTER_TYPE_OCV3


} // end namespace ocv
} // end namespace maptk
} // end namespace kwiver
