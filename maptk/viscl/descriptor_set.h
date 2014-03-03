/*ckwg +5
 * Copyright 2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#ifndef MAPTK_VISCL_DESCRIPTOR_SET_H_
#define MAPTK_VISCL_DESCRIPTOR_SET_H_


#include <maptk/core/descriptor_set.h>
#include <maptk/viscl/viscl_config.h>
#include <viscl/core/buffer.h>

namespace maptk
{

namespace vcl
{

/// A concrete descriptor set that wraps VisCL descriptors.
class MAPTK_VISCL_EXPORT descriptor_set
: public maptk::descriptor_set
{
public:

  /// Default Constructor
  descriptor_set() {}

  /// Constructor from VisCL descriptors
  explicit descriptor_set(const viscl::buffer& viscl_descriptors)
  : data_(viscl_descriptors) {}

  /// Return the number of descriptor in the set
  virtual size_t size() const { return data_.len(); }

  /// Return a vector of descriptor shared pointers
  /**
    * Warning: These descriptors must be matched by hamming distance
    */
  virtual std::vector<descriptor_sptr> descriptors() const;

  /// Return the native VisCL descriptors structure
  const viscl::buffer& viscl_descriptors() const { return data_; }

protected:

  /// The handle to a VisCL set of descriptors
  viscl::buffer data_;
};


/// Convert a descriptor set to a VisCL descriptor set must be <int,4>
MAPTK_VISCL_EXPORT viscl::buffer
descriptors_to_viscl(const maptk::descriptor_set& desc_set);


} // end namespace vcl

} // end namespace maptk


#endif // MAPTK_VISCL_DESCRIPTOR_SET_H_
