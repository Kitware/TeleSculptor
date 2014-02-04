/*ckwg +5
 * Copyright 2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#include <maptk/viscl/extract_descriptors.h>
#include <maptk/viscl/image_container.h>
#include <maptk/viscl/feature_set.h>
#include <maptk/viscl/descriptor_set.h>

#include <viscl/tasks/BRIEF.h>

namespace maptk
{

namespace vcl
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

  viscl::brief<10> brief;
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

  viscl::image img = vcl::image_container_to_viscl(*image_data);
  vcl::feature_set::type fs = vcl::features_to_viscl(*features, img.width(), img.height());
  viscl::buffer descriptors;
  d_->brief.compute_descriptors(img, fs.features_, features->size(), descriptors);
  return descriptor_set_sptr(new vcl::descriptor_set(descriptors));
}


} // end namespace viscl

} // end namespace maptk
