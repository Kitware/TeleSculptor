/*ckwg +5
 * Copyright 2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#include <maptk/viscl/descriptor_set.h>


namespace maptk
{

namespace viscl
{

/// Return a vector of descriptor shared pointers
std::vector<descriptor_sptr>
descriptor_set
::descriptors() const
{
  std::vector<descriptor_sptr> desc;
  // TODO download/convert descriptors from GPU
  return desc;
}


/// Convert any descriptor set a VisCL descriptor set
// TODO function to convert/upload descriptors to GPU
//type
//descriptors_to_viscl(const maptk::descriptor_set& desc_set);
//{
//  return type;
//}


} // end namespace viscl

} // end namespace maptk
