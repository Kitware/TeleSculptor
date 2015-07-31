/*ckwg +29
 * Copyright 2014-2015 by Kitware, Inc.
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

#include <algorithm>

#include <boost/foreach.hpp>

#include <maptk/plugins/viscl/match_set.h>

#include <viscl/core/manager.h>


namespace kwiver {
namespace maptk {

namespace vcl
{

/// Return the number of matches in the set
size_t
match_set
::size() const
{
  std::vector<int> viscl_matches(data_.len());
  viscl::cl_queue_t queue = viscl::manager::inst()->create_queue();
  queue->enqueueReadBuffer(*data_().get(), CL_TRUE, 0, data_.mem_size(), &viscl_matches[0]);
  queue->finish();

  size_t count = 0;
  for (unsigned int i = 0; i < viscl_matches.size(); i++)
  {
    if (viscl_matches[i] > -1)
    {
      count++;
    }
  }

  return count;
}

/// Return a vector of matching indices
std::vector<vital::match>
match_set
::matches() const
{
  std::vector<vital::match> m;

  std::vector<int> viscl_matches(data_.len());
  viscl::cl_queue_t queue = viscl::manager::inst()->create_queue();
  queue->enqueueReadBuffer(*data_().get(), CL_TRUE, 0, data_.mem_size(), &viscl_matches[0]);
  queue->finish();

  for (unsigned int i = 0; i < viscl_matches.size(); i++)
  {
    if (viscl_matches[i] > -1)
    {
      m.push_back(vital::match(viscl_matches[i], i));
    }
  }

  return m;
}

/// Convert any match set to VisCL matches
viscl::buffer
matches_to_viscl(const vital::match_set& m_set)
{
  if( const vcl::match_set* m_viscl =
          dynamic_cast<const vcl::match_set*>(&m_set) )
  {
    return m_viscl->viscl_matches();
  }

  const std::vector<vital::match> mats = m_set.matches();

  unsigned int maxindex = 0;
  for (unsigned int i = 0; i < mats.size(); i++)
  {
    if (mats[i].second > maxindex)
    {
      maxindex = mats[i].second;
    }
  }

  std::vector<int> buf(maxindex + 1, -1);
  for (unsigned int i = 0; i < mats.size(); i++)
  {
    buf[mats[i].second] = mats[i].first;
  }

  viscl::buffer viscl_data = viscl::manager::inst()->create_buffer<int>(CL_MEM_READ_WRITE, buf.size());
  viscl::cl_queue_t queue = viscl::manager::inst()->create_queue();
  queue->enqueueWriteBuffer(*viscl_data().get(), CL_TRUE, 0, viscl_data.mem_size(), &buf[0]);
  queue->finish();

  return viscl_data;
}


} // end namespace vcl

} // end namespace maptk
} // end namespace kwiver
