/*ckwg +29
 * Copyright 2016 by Kitware, Inc.
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
 * \brief OCV simple blob feature detector wrapper implementation
 */

#include "detect_features_simple_blob.h"

using namespace kwiver::vital;

namespace kwiver {
namespace maptk {
namespace ocv {


class detect_features_simple_blob::priv
{
public:
  /// Constructor
  priv()
    : p()
  {
  }

  /// Copy constructor
  priv( priv const &other )
    : p( other.p )
  {
  }

  /// Create new algorithm based on current parameter values
  cv::Ptr<cv::SimpleBlobDetector> create() const
  {
#ifndef MAPTK_HAS_OPENCV_VER_3
    return cv::Ptr<cv::SimpleBlobDetector>(
      new cv::SimpleBlobDetector( p )
    );
#else
    return cv::SimpleBlobDetector::create( p );
#endif
  }

  /// Update config block with current parameters and values
  void update_config( config_block_sptr config ) const
  {
    config->set_value( "threshold_step", p.thresholdStep,
                       "Defines stepping between min and max threshold when "
                       "converting the source image to binary images by "
                       "applying thresholding with several thresholds from "
                       "min_threshold (inclusive) to max_threshold (exclusive) "
                       " with distance threshold_step between neighboring "
                       "thresholds." );
    config->set_value( "threshold_min", p.minThreshold );
    config->set_value( "threshold_max", p.maxThreshold );
    config->set_value( "min_repeatability", p.minRepeatability );
    config->set_value( "min_dist_between_blocks", p.minDistBetweenBlobs,
                       "Close centers form one group that corresponds to one "
                       "blob, controlled by this distance value." );

    config->set_value( "filter_by_color", p.filterByColor,
                       "Enable blob filtering by intensity of the binary image "
                       "at the center of the blob to blob_color. If they "
                       "differ, the blob is filtered out. Use blob_color = 0 "
                       "to extract dark blobs and blob_color = 255 to extract "
                       "light blobs" );
    config->set_value( "blob_color", p.blobColor );

    config->set_value( "filter_by_area", p.filterByArea,
                       "Enable blob filtering by area to those between "
                       "min_area (inclusive) and max_area (exclusive)." );
    config->set_value( "min_area", p.minArea );
    config->set_value( "max_area", p.maxArea );

    config->set_value( "filter_by_circularity", p.filterByCircularity,
                       "Enable blob filtering by circularity to those between "
                       "min_circularity (inclusive) and max_circularity "
                       "(exclusive)." );
    config->set_value( "min_circularity", p.minCircularity );
    config->set_value( "max_circularity", p.maxCircularity );

    config->set_value( "filter_by_inertia", p.filterByInertia,
                       "Enable blob filtering by the ratio of inertia between "
                       "min_inertia_ratio (inclusive) and max_inertia_ratio "
                       "(exclusive)." );
    config->set_value( "min_inertia_ratio", p.minInertiaRatio );
    config->set_value( "max_inertia_ratio", p.maxInertiaRatio );

    config->set_value( "filter_by_convexity", p.filterByConvexity,
                       "Enable filtering by convexity where blobs have "
                       "convexity (area / area of blob convex hull) between "
                       "min_convexity (inclusive) and max_convexity "
                       "(exclusive)." );
    config->set_value( "min_convexity", p.minConvexity );
    config->set_value( "max_convexity", p.maxConvexity );
  }

  /// Set the current parameter values based on the given config block
  void set_config( config_block_sptr config )
  {
    p.thresholdStep = config->get_value<float>( "threshold_step" );
    p.minThreshold = config->get_value<float>( "threshold_min" );
    p.maxThreshold = config->get_value<float>( "threshold_max" );
    p.minRepeatability = config->get_value<std::size_t>( "min_repeatability" );
    p.minDistBetweenBlobs = config->get_value<float>( "min_dist_between_blocks" );

    p.filterByColor = config->get_value<bool>( "filter_by_color" );
    p.blobColor = config->get_value<unsigned char>( "blob_color" );

    p.filterByArea = config->get_value<bool>( "filter_by_area" );
    p.minArea = config->get_value<float>( "min_area" );
    p.maxArea = config->get_value<float>( "max_area" );

    p.filterByCircularity = config->get_value<bool>( "filter_by_circularity" );
    p.minCircularity = config->get_value<float>( "min_circularity" );
    p.maxCircularity = config->get_value<float>( "max_circularity" );

    p.filterByInertia = config->get_value<bool>( "filter_by_inertia" );
    p.minInertiaRatio = config->get_value<float>( "min_inertia_ratio" );
    p.maxInertiaRatio = config->get_value<float>( "max_inertia_ratio" );

    p.filterByConvexity = config->get_value<bool>( "filter_by_convexity" );
    p.minConvexity = config->get_value<float>( "min_convexity" );
    p.maxConvexity = config->get_value<float>( "max_convexity" );
  }

  cv::SimpleBlobDetector::Params p;
};


detect_features_simple_blob
::detect_features_simple_blob()
  : p_( new priv )
{
  attach_logger( "maptk.ocv.simple_blob_detector" );
  detector = p_->create();
}


detect_features_simple_blob
::detect_features_simple_blob(const detect_features_simple_blob &other)
  : p_( new priv( *other.p_ ) )
{
  attach_logger( "maptk.ocv.simple_blob_detector" );
  detector = p_->create();
}


detect_features_simple_blob
::~detect_features_simple_blob()
{
}


vital::config_block_sptr
detect_features_simple_blob
::get_configuration() const
{
  config_block_sptr config = ocv::detect_features::get_configuration();
  p_->update_config( config );
  return config;
}


void
detect_features_simple_blob
::set_configuration(vital::config_block_sptr config)
{
  config_block_sptr c = get_configuration();
  c->merge_config( config );
  p_->set_config( c );
  detector = p_->create();
}


bool
detect_features_simple_blob
::check_configuration(vital::config_block_sptr config) const
{
  return true;
}


} // end namespace kwiver
} // end namespace maptk
} // end namespace ocv
