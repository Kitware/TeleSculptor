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


namespace maptk
{

namespace viscl
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

  // TODO add custom VisCL data here
};


/// Constructor
match_features
::match_features()
: d_(new priv)
{
}


/// Copy Constructor
match_features
::match_features(const match_features& other)
: d_(new priv(*other.d_))
{
}


/// Destructor
match_features
::~match_features()
{
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
  // TODO convert to VisCL matches
  type d1 = descriptors_to_viscl(*desc1);
  type d2 = descriptors_to_viscl(*desc2);
  // TODO compute matches
  return match_set_sptr(new match_set(matches));
}


} // end namespace viscl

} // end namespace maptk
