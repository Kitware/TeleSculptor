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
std::vector<match>
match_set
::matches() const
{
  std::vector<match> m;

  std::vector<int> viscl_matches(data_.len());
  viscl::cl_queue_t queue = viscl::manager::inst()->create_queue();
  queue->enqueueReadBuffer(*data_().get(), CL_TRUE, 0, data_.mem_size(), &viscl_matches[0]);
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

/// Convert any match set to VisCL matches
viscl::buffer
matches_to_viscl(const maptk::match_set& m_set)
{
  if( const vcl::match_set* m_viscl =
          dynamic_cast<const vcl::match_set*>(&m_set) )
  {
    return m_viscl->viscl_matches();
  }

  const std::vector<match> mats = m_set.matches();

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
