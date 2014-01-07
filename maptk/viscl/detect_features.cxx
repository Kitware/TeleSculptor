/*ckwg +5
 * Copyright 2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#include "detect_features.h"

#include <vector>
#include <maptk/viscl/feature_set.h>
#include <maptk/viscl/image_container.h>


namespace maptk
{

namespace viscl
{


/// Private implementation class
class detect_features::priv
{
public:
  /// Constructor
  priv()
  {
  }

  /// Copy Constructor
  priv(const priv& other)
  {
  }

  /// TODO define any private VisCL data needed
};


/// Constructor
detect_features
::detect_features()
: d_(new priv)
{
}


/// Copy Constructor
detect_features
::detect_features(const detect_features& other)
: d_(new priv(*other.d_))
{
}


/// Destructor
detect_features
::~detect_features()
{
}


/// Extract a set of image features from the provided image
/// \param image_data contains the image data to process
/// \returns a set of image features
feature_set_sptr
detect_features
::detect(image_container_sptr image_data) const
{
  // TODO convert the image data to VisCL
  // TODO run feature detection
  // TODO return a viscl::feature_set
  return feature_set_sptr(new feature_set(/* TODO data */));
}


} // end namespace viscl

} // end namespace maptk
