/*ckwg +5
 * Copyright 2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#ifndef MAPTK_VISCL_DESCRIPTOR_SET_H_
#define MAPTK_VISCL_DESCRIPTOR_SET_H_


#include <maptk/core/descriptor_set.h>


namespace maptk
{

namespace viscl
{

/// A concrete descriptor set that wraps VisCL descriptors.
class descriptor_set
: public maptk::descriptor_set
{
public:
  /// Default Constructor
  descriptor_set() {}

  /// Constructor from VisCL descriptors
  // TODO implement constructor
  //explicit descriptor_set(const type& viscl_descriptors)
  //: data_(data_descriptors) {}

  /// Return the number of descriptor in the set
  virtual size_t size() const { return 0; } //TODO number of descriptors

  /// Return a vector of descriptor shared pointers
  virtual std::vector<descriptor_sptr> descriptors() const;

  /// Return the native VisCL descriptors structure
  // TODO define this
  //const type& viscl_descriptors() const { return data_; }

protected:

  /// The handle to a VisCL set of descriptors
  // TODO define this variable
  // type data_;
};


/// Convert any descriptor set a VisCL descriptor set
// TODO function to convert/upload descriptors to GPU
//type
//descriptors_to_viscl(const maptk::descriptor_set& desc_set);


} // end namespace viscl

} // end namespace maptk


#endif // MAPTK_VISCL_DESCRIPTOR_SET_H_
