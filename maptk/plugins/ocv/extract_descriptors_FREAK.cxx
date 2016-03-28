//
// Created by Paul Tunison on 3/27/16.
//

#ifndef MAPTK_HAS_OPENCV_VER_3
#include "extract_descriptors_FREAK.h"

namespace kwiver {
namespace maptk {
namespace ocv {


class extract_descriptors_FREAK::priv
{
public:
  /// Constructor
  priv()
    : orientation_normalized( true ),
      scale_normalized( true ),
      pattern_scale( 22.0f ),
      n_octaves( 4 )
  {
  }

  /// Copy constructor
  priv( priv const &other )
      : orientation_normalized( other.orientation_normalized ),
        scale_normalized( other.scale_normalized ),
        pattern_scale( other.pattern_scale ),
        n_octaves( other.n_octaves )
  {
  }

  /// Create new cv::Ptr algo instance
  cv::Ptr<cv::FREAK> create() const
  {
    return cv::Ptr<cv::FREAK>(
        new cv::FREAK( orientation_normalized, scale_normalized, pattern_scale,
                       n_octaves )
    );
  }

  /// Set current parameter values to the given config block
  void update_config( vital::config_block_sptr &config ) const
  {
    config->set_value( "orientation_normalized", orientation_normalized,
                       "enable orientation normalization" );
    config->set_value( "scale_normalized", scale_normalized,
                       "enable scale normalization" );
    config->set_value( "pattern_scale", pattern_scale,
                       "scaling of the description pattern" );
    config->set_value( "n_octaves", n_octaves,
                       "number of octaves covered by the detected keypoints" );
  }

  /// Set our parameters based on the given config block
  void set_config( vital::config_block_sptr const &config )
  {
    orientation_normalized = config->get_value<bool>("orientation_normalized");
    scale_normalized = config->get_value<bool>("scale_normalized");
    pattern_scale = config->get_value<float>("pattern_scale");
    n_octaves = config->get_value<int>("n_octaves");
  }

  /// Update algorithm instance with current parameter values
  void update_algo( cv::Ptr<cv::FREAK> freak ) const
  {
    freak->set( "orientationNormalized", orientation_normalized );
    freak->set( "scaleNormalized", scale_normalized );
    freak->set( "patternScale", pattern_scale );
    freak->set( "nbOctave", n_octaves );
  }

  /// Params
  bool orientation_normalized;
  bool scale_normalized;
  float pattern_scale;
  int n_octaves;
};


/// Constructor
extract_descriptors_FREAK
::extract_descriptors_FREAK()
    : p_( new priv )
{
  attach_logger("maptk.ocv.FREAK");
  extractor = p_->create();
}


/// Copy Constructor
extract_descriptors_FREAK
::extract_descriptors_FREAK(extract_descriptors_FREAK const &other)
: p_( new priv(*other.p_) )
{
  attach_logger("maptk.ocv.FREAK");
  extractor = p_->create();
}


/// Destructor
extract_descriptors_FREAK
::~extract_descriptors_FREAK()
{
}


vital::config_block_sptr
extract_descriptors_FREAK
::get_configuration() const
{
  vital::config_block_sptr config = extract_descriptors::get_configuration();
  p_->update_config(config);
  return config;
}


void
extract_descriptors_FREAK
::set_configuration(vital::config_block_sptr in_config)
{
  vital::config_block_sptr config = get_configuration();
  config->merge_config( in_config );
  p_->set_config( in_config );
  p_->update_algo( extractor );
}


bool
extract_descriptors_FREAK
::check_configuration(vital::config_block_sptr in_config) const
{
  return true;
}


}
}
}

#endif //MAPTK_HAS_OPENCV_VER_3
