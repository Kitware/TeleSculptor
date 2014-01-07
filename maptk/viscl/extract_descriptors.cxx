/*ckwg +5
 * Copyright 2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#include <maptk/viscl/extract_descriptors.h>
#include <maptk/viscl/image_container.h>
#include <maptk/viscl/feature_set.h>
#include <maptk/viscl/descriptor_set.h>


namespace maptk
{

namespace viscl
{

/// Private implementation class
class extract_descriptors::priv
{
public:
  /// Constructor
  priv()
  {
  }

  /// Copy constructor
  priv(const priv& other)
  {
  }

  /// TODO any VISCL data needed goes here
};


/// Constructor
extract_descriptors
::extract_descriptors()
: d_(new priv)
{
}


/// Copy Constructor
extract_descriptors
::extract_descriptors(const extract_descriptors& other)
: d_(new priv(*other.d_))
{
}


/// Destructor
extract_descriptors
::~extract_descriptors()
{
}


/// Extract from the image a descriptor corresoponding to each feature
descriptor_set_sptr
extract_descriptors
::extract(image_container_sptr image_data,
          feature_set_sptr features) const
{
  if( !image_data || !features )
  {
    return descriptor_set_sptr();
  }
  // TODO extract image form image_data, upload if needed
  // TODO extract features data, upload if needed
  // TODO extract descriptors
  return descriptor_set_sptr(new viscl::descriptor_set(/* TODO data */));
}


} // end namespace viscl

} // end namespace maptk
