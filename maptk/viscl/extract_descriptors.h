/*ckwg +5
 * Copyright 2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#ifndef MAPTK_VISCL_EXTRACT_DESCRIPTORS_H_
#define MAPTK_VISCL_EXTRACT_DESCRIPTORS_H_

#include <maptk/core/algo/extract_descriptors.h>
#include <boost/scoped_ptr.hpp>

namespace maptk
{

namespace viscl
{

/// An class for extracting feature descriptors using VisCL
class extract_descriptors
: public algo::algorithm_impl<extract_descriptors, algo::extract_descriptors>
{
public:
  /// Default Constructor
  extract_descriptors();

  /// Destructor
  virtual ~extract_descriptors();

  /// Copy Constructor
  extract_descriptors(const extract_descriptors& other);

  /// Return the name of this implementation
  std::string impl_name() const { return "viscl"; }

  // No configuration for this class yet
  virtual void set_configuration(config_block_sptr /*config*/) { }
  virtual bool check_configuration(config_block_sptr /*config*/) const { return true; }

  /// Extract from the image a descriptor corresoponding to each feature
  /// \param image_data contains the image data to process
  /// \param features the feature locations at which descriptors are extracted
  /// \returns a set of feature descriptors
  virtual descriptor_set_sptr
  extract(image_container_sptr image_data,
          feature_set_sptr features) const;

private:
  /// private implementation class
  class priv;
  boost::scoped_ptr<priv> d_;
};

} // end namespace viscl

} // end namespace maptk


#endif // MAPTK_VISCL_EXTRACT_DESCRIPTORS_H_
