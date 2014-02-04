/*ckwg +5
 * Copyright 2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#include <maptk/viscl/descriptor_set.h>
#include <viscl/core/manager.h>


namespace maptk
{

namespace vcl
{

/// Return a vector of descriptor shared pointers
std::vector<descriptor_sptr>
descriptor_set
::descriptors() const
{
  using namespace maptk;
  std::vector<descriptor_sptr> desc;

  cl_int4 *buf = new cl_int4[data_.len()];

  viscl::cl_queue_t queue = queue = viscl::manager::inst()->create_queue();
  queue->enqueueReadBuffer(*data_().get(), CL_TRUE, 0, data_.mem_size(), buf);
  queue->finish();

  for (unsigned int i = 0; i < data_.len(); i++)
  {
    descriptor_fixed<int,4> *d = new descriptor_fixed<int,4>;
    memcpy(d->raw_data(), &buf[i].s, sizeof(cl_int4));
    desc.push_back(descriptor_sptr(d));
  }

  delete [] buf;

  return desc;
}


//viscl cannot take an arbitrary descriptor so this function is not
//implemented when type is not viscl, could check if type is <int,4> and
//convert
viscl::buffer
descriptors_to_viscl(const maptk::descriptor_set& desc_set)
{
  if( const vcl::descriptor_set* m_viscl =
          dynamic_cast<const vcl::descriptor_set*>(&desc_set) )
  {
    return m_viscl->viscl_descriptors();
  }

  //TODO: throw exception
  return viscl::buffer();
}


} // end namespace viscl

} // end namespace maptk
