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
 * \brief OCV ORB feature detector and extractor wrapper implementation
 */

#include "feature_detect_extract_ORB.h"

namespace kwiver {
namespace maptk {
namespace ocv {

using namespace kwiver::vital;


namespace {

/// Common ORB private implementation class
class priv
{
public:
  /// Constructor
  priv()
     : n_features( 500 )
     , scale_factor( 1.2f )
     , n_levels( 8 )
     , edge_threshold( 31 )
     , first_level( 0 )
     , wta_k( 2 )
     , score_type( cv::ORB::HARRIS_SCORE )
     , patch_size( 31 )
#ifdef MAPTK_HAS_OPENCV_VER_3
     , fast_threshold( 20 )
#endif
  {
  }

  /// Copy Constructor
  priv(priv const &other)
     : n_features( other.n_features )
     , scale_factor( other.scale_factor )
     , n_levels( other.n_levels )
     , edge_threshold( other.edge_threshold )
     , first_level( other.first_level )
     , wta_k( other.wta_k )
     , score_type( other.score_type )
     , patch_size( other.patch_size )
#ifdef MAPTK_HAS_OPENCV_VER_3
     , fast_threshold( other.fast_threshold )
#endif
  {
  }

  /// Create new impl instance based on current parameters
  cv::Ptr<cv::ORB> create() const
  {
#ifndef MAPTK_HAS_OPENCV_VER_3
    return cv::Ptr<cv::ORB>(
       new cv::ORB( n_features, scale_factor, n_levels, edge_threshold,
                            first_level, wta_k, score_type, patch_size )
    );
#else
    return cv::ORB::create( n_features, scale_factor, n_levels, edge_threshold,
                            first_level, wta_k, score_type, patch_size,
                            fast_threshold );
#endif
  }

  /// Update given configuration with current parameter keys and values
  void update_configuration(vital::config_block_sptr &config) const
  {
    config->set_value( "n_features", n_features,
                       "The maximum number of features to retain" );
    config->set_value( "scale_factor", scale_factor,
                       "Pyramid decimation ratio, greater than 1. "
                           "scaleFactor==2 means the classical pyramid, where each "
                           "next level has 4x less pixels than the previous, but "
                           "such a big scale factor will degrade feature matching "
                           "scores dramatically. On the other hand, too close to 1 "
                           "scale factor will mean that to cover certain scale "
                           "range you will need more pyramid levels and so the "
                           "speed will suffer." );
    config->set_value( "n_levels", n_levels,
                       "The number of pyramid levels. The smallest level will "
                           "have linear size equal to "
                           "input_image_linear_size/pow(scale_factor, "
                           "n_levels)." );
    config->set_value( "edge_threshold", edge_threshold,
                       "This is size of the border where the features are not "
                           "detected. It should roughly match the patch_size "
                           "parameter." );
    config->set_value( "first_level", first_level,
                       "It should be 0 in the current implementation." );
    config->set_value( "wta_k", wta_k,
                       "The number of points that produce each element of the "
                           "oriented BRIEF descriptor. The default value 2 "
                           "means the BRIEF where we take a random point pair "
                           "and compare their brightnesses, so we get 0/1 "
                           "response. Other possible values are 3 and 4. For "
                           "example, 3 means that we take 3 random points (of "
                           "course, those point coordinates are random, but "
                           "they are generated from the pre-defined seed, so "
                           "each element of BRIEF descriptor is computed "
                           "deterministically from the pixel rectangle), find "
                           "point of maximum brightness and output index of "
                           "the winner (0, 1 or 2). Such output will occupy 2 "
                           "bits, and therefore it will need a special variant "
                           "of Hamming distance, denoted as NORM_HAMMING2 (2 "
                           "bits per bin). When WTA_K=4, we take 4 random "
                           "points to compute each bin (that will also occupy "
                           "2 bits with possible values 0, 1, 2 or 3)." );
    std::stringstream ss;
    ss << "The default HARRIS_SCORE (value=" << cv::ORB::HARRIS_SCORE << ") "
       << "means that Harris algorithm is used to rank features (the score is "
       << "written to KeyPoint::score and is used to retain best n_features "
       << "features); FAST_SCORE (value=" << cv::ORB::FAST_SCORE << ") is "
       << "alternative value of the parameter that produces slightly less "
       << "stable key-points, but it is a little faster to compute.";
    config->set_value( "score_type", score_type, ss.str() );
    config->set_value( "patch_size", patch_size,
                       "Size of the patch used by the oriented BRIEF "
                           "descriptor. Of course, on smaller pyramid layers "
                           "the perceived image area covered by a feature will "
                           "be larger." );
#ifdef MAPTK_HAS_OPENCV_VER_3
    config->set_value( "fast_threshold", fast_threshold, "Undocumented" );
#endif
  }

  /// Update parameters based on the given config-block
  void set_configuration(vital::config_block_sptr const &config)
  {
    n_features = config->get_value<int>("n_features");
    scale_factor = config->get_value<float>("scale_factor");
    n_levels = config->get_value<int>("n_levels");
    edge_threshold = config->get_value<int>("edge_threshold");
    first_level = config->get_value<int>("first_level");
    wta_k = config->get_value<int>("wta_k");
    score_type = config->get_value<int>("score_type");
    patch_size = config->get_value<int>("patch_size");
#ifdef MAPTK_HAS_OPENCV_VER_3
    fast_threshold = config->get_value<int>("fast_threshold");
#endif
  }

  /// Update algo with current parameter values
  void update_algo(cv::Ptr<cv::ORB> orb) const
  {
#ifndef MAPTK_HAS_OPENCV_VER_3
    orb->set( "nFeatures", n_features );
    orb->set( "scaleFactor", scale_factor );
    orb->set( "nLevels", n_levels );
    orb->set( "firstLevel", first_level );
    orb->set( "edgeThreshold", edge_threshold );
    orb->set( "patchSize", patch_size );
    orb->set( "WTA_K", wta_k );
    orb->set( "scoreType", score_type );
#else
    orb->setMaxFeatures( n_features );
    orb->setScaleFactor( scale_factor );
    orb->setNLevels( n_levels );
    orb->setEdgeThreshold( edge_threshold );
    orb->setFirstLevel( first_level );
    orb->setWTA_K( wta_k );
    orb->setScoreType( score_type );
    orb->setPatchSize( patch_size );
    orb->setFastThreshold( fast_threshold );
#endif
  }

  /// Parameters
  int n_features;
  float scale_factor;
  int n_levels;
  int edge_threshold;
  int first_level;
  int wta_k;
  int score_type;
  int patch_size;
#ifdef MAPTK_HAS_OPENCV_VER_3
  int fast_threshold;
#endif
};

} // end anon namespace


detect_features_ORB
::detect_features_ORB()
   : p_( new priv )
{
  attach_logger("maptk.ocv.ORB");
  detector = p_->create();
}


detect_features_ORB
::detect_features_ORB(detect_features_ORB const &other)
   : p_( new priv( *other.p_ ) )
{
  attach_logger("maptk.ocv.ORB");
  detector = p_->create();
}


detect_features_ORB
::~detect_features_ORB()
{
}


vital::config_block_sptr
detect_features_ORB
::get_configuration() const
{
  config_block_sptr config = detect_features::get_configuration();
  p_->update_configuration(config);
  return config;
}


void
detect_features_ORB
::set_configuration(vital::config_block_sptr config)
{
  config_block_sptr c = get_configuration();
  c->merge_config(config);
  p_->set_configuration(c);
#ifndef MAPTK_HAS_OPENCV_VER_3
  p_->update_algo( detector );
#else
  p_->update_algo( detector.dynamicCast<cv::ORB>() );
#endif
}


bool
detect_features_ORB
::check_configuration(vital::config_block_sptr config) const
{
  return true;
}


extract_descriptors_ORB
::extract_descriptors_ORB()
   : p_( new priv )
{
  attach_logger("maptk.ocv.ORB");
  extractor = p_->create();
}


extract_descriptors_ORB
::extract_descriptors_ORB(extract_descriptors_ORB const &other)
   : p_( new priv( *other.p_ ) )
{
  attach_logger("maptk.ocv.ORB");
  extractor = p_->create();
}


extract_descriptors_ORB
::~extract_descriptors_ORB()
{
}


vital::config_block_sptr
extract_descriptors_ORB
::get_configuration() const
{
  config_block_sptr config = extract_descriptors::get_configuration();
  p_->update_configuration(config);
  return config;
}


void
extract_descriptors_ORB
::set_configuration(vital::config_block_sptr config)
{
  config_block_sptr c = get_configuration();
  c->merge_config(config);
  p_->set_configuration(c);
#ifndef MAPTK_HAS_OPENCV_VER_3
  p_->update_algo( extractor );
#else
  p_->update_algo( extractor.dynamicCast<cv::ORB>() );
#endif
}


bool
extract_descriptors_ORB
::check_configuration(vital::config_block_sptr config) const
{
  return true;
}


} // end namespace kwiver
} // end namespace maptk
} // end namespace ocv
