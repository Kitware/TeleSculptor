/*ckwg +29
 * Copyright 2014 by Kitware, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither name of Kitware, Inc. nor the names of any contributors may be used
 *    to endorse or promote products derived from this software without specific
 *    prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS ``AS IS''
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHORS OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "descriptor_set.h"

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

  viscl::cl_queue_t queue = viscl::manager::inst()->create_queue();
  queue->enqueueReadBuffer(*data_().get(), CL_TRUE, 0, data_.mem_size(), buf);
  queue->finish();

  for (unsigned int i = 0; i < data_.len(); i++)
  {
    descriptor_fixed<int,4> *d = new descriptor_fixed<int,4>;
    memcpy(d->raw_data(), &buf[i].s, sizeof(int)*4);
    desc.push_back(descriptor_sptr(d));
  }

  delete [] buf;

  return desc;
}


/// Convert a descriptor set to a VisCL descriptor set must be <int,4>
viscl::buffer
descriptors_to_viscl(const maptk::descriptor_set& desc_set)
{
  if( const vcl::descriptor_set* m_viscl =
          dynamic_cast<const vcl::descriptor_set*>(&desc_set) )
  {
    return m_viscl->viscl_descriptors();
  }

  //viscl cannot take an arbitrary descriptor so this function
  //only checks for <int,4> descriptors
  std::vector<cl_int4> viscl_descr;
  std::vector<descriptor_sptr> descriptors = desc_set.descriptors();
  for (unsigned int i = 0; i < descriptors.size(); i++)
  {
    //check if type is <int,4> if not we are done
    if ( const descriptor_fixed<int,4> * dfixed =
          dynamic_cast<const descriptor_fixed<int,4> *>(descriptors[i].get()) )
    {
      cl_int4 d;
      memcpy(&d.s, dfixed->raw_data(), sizeof(int)*4);
      viscl_descr.push_back(d);
    }
    else
    {
      break;
    }
  }

  if (viscl_descr.size() == descriptors.size())
  {
    viscl::buffer buf = viscl::manager::inst()->create_buffer<cl_int4>(CL_MEM_READ_WRITE, viscl_descr.size());
    viscl::cl_queue_t queue = viscl::manager::inst()->create_queue();
    queue->enqueueWriteBuffer(*buf().get(), CL_TRUE, 0, buf.mem_size(), &viscl_descr[0]);
    queue->finish();
    return buf;
  }

  //TODO: throw exception
  return viscl::buffer();
}


} // end namespace vcl

} // end namespace maptk
