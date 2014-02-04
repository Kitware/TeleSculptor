/*ckwg +5
 * Copyright 2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#include "match_features.h"

#include <vector>
#include <maptk/viscl/feature_set.h>
#include <maptk/viscl/descriptor_set.h>
#include <maptk/viscl/match_set.h>

#include <viscl/tasks/track_descr_match.h>

namespace maptk
{

namespace vcl
{


/// Private implementation class
class match_features::priv
{
public:
  /// Constructor
  priv()
  {
  }

  // Copy Constructor
  priv(const priv& other)
  {
  }

  viscl::track_descr_match matcher;
};


/// Constructor
match_features
::match_features()
: d_(new priv), imgwidth_(0), imgheight_(0)
{
}


/// Copy Constructor
match_features
::match_features(const match_features& other)
: d_(new priv(*other.d_)), imgwidth_(other.imgwidth_), imgheight_(other.imgheight_)
{
}


/// Destructor
match_features
::~match_features()
{
}

void
match_features
::set_img_dimensions(unsigned int width, unsigned int height)
{
  imgwidth_ = width;
  imgheight_ = height;
}

/// Match one set of features and corresponding descriptors to another
match_set_sptr
match_features
::match(feature_set_sptr feat1, descriptor_set_sptr desc1,
        feature_set_sptr feat2, descriptor_set_sptr desc2) const
{
  if( !desc1 || !desc2 )
  {
    return match_set_sptr();
  }

  viscl::buffer d1 = descriptors_to_viscl(*desc1);
  viscl::buffer d2 = descriptors_to_viscl(*desc2);
  vcl::feature_set::type f1 = vcl::features_to_viscl(*feat1, imgwidth_, imgheight_);
  vcl::feature_set::type f2 = vcl::features_to_viscl(*feat2, imgwidth_, imgheight_);

  size_t numkpts2 = feat2->size();
  viscl::buffer matches = d_->matcher.match(f1.features_, f1.kptmap_, d1,
                                            f2.features_, numkpts2, f2.kptmap_, d2);

  return match_set_sptr(new match_set(matches));
}


} // end namespace viscl

} // end namespace maptk
