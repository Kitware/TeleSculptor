/*ckwg +5
 * Copyright 2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */


#include <maptk/viscl/match_set.h>
#include <boost/foreach.hpp>
#include <algorithm>

#include <viscl/core/manager.h>

namespace maptk
{

namespace vcl
{

/// Return a vector of matching indices
std::vector<match>
match_set
::matches() const
{
  std::vector<match> m;

  std::vector<int> viscl_matches(data_.len());
  viscl::cl_queue_t queue = queue = viscl::manager::inst()->create_queue();
  queue->enqueueWriteBuffer(*data_().get(), CL_TRUE, 0, data_.mem_size(), &viscl_matches[0]);
  queue->finish();

  for (unsigned int i = 0; i < viscl_matches.size(); i++)
  {
    if (viscl_matches[i] > -1)
    {
      m.push_back(match(viscl_matches[i], i));
    }
  }

  return m;
}

// Convert any match set to VisCL matches
// Multiple matches with a kpt from the 2nd set will not be kept
viscl::buffer
matches_to_viscl(const maptk::match_set& m_set, size_t numkpts2)
{
  if( const vcl::match_set* m_viscl =
          dynamic_cast<const vcl::match_set*>(&m_set) )
  {
    return m_viscl->viscl_matches();
  }

  const std::vector<match> mats = m_set.matches();

  int *buf = new int[numkpts2];
  memset(buf, -1, numkpts2);
  for (unsigned int i = 0; i < mats.size(); i++)
  {
    buf[mats[i].second] = mats[i].first;
  }

  viscl::buffer viscl_data = viscl::manager::inst()->create_buffer<int>(CL_MEM_READ_WRITE, mats.size());
  viscl::cl_queue_t queue = queue = viscl::manager::inst()->create_queue();
  queue->enqueueWriteBuffer(*viscl_data().get(), CL_TRUE, 0, viscl_data.mem_size(), buf);
  queue->finish();

  delete [] buf;
  return viscl_data;
}


} // end namespace viscl

} // end namespace maptk
