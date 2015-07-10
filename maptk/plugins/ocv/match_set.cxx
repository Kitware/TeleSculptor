/*ckwg +29
 * Copyright 2013-2015 by Kitware, Inc.
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

/**
 * \file
 * \brief OCV match_set implementation
 */

#include "match_set.h"

#include <boost/foreach.hpp>


namespace maptk
{

namespace ocv
{

/// Return a vector of matching indices
std::vector<kwiver::vital::match>
match_set
::matches() const
{
  std::vector<kwiver::vital::match> m;
  BOOST_FOREACH(cv::DMatch dm, this->data_)
  {
    m.push_back( kwiver::vital::match(dm.queryIdx, dm.trainIdx));
  }
  return m;
}


/// Convert any match set to a vector of OpenCV cv::DMatch
std::vector<cv::DMatch>
matches_to_ocv_dmatch(const kwiver::vital::match_set& m_set)
{
  if( const ocv::match_set* m_ocv =
          dynamic_cast<const ocv::match_set*>(&m_set) )
  {
    return m_ocv->ocv_matches();
  }
  std::vector<cv::DMatch> dm;
  const std::vector<kwiver::vital::match> mats = m_set.matches();
  BOOST_FOREACH( kwiver::vital::match m, mats)
  {
    dm.push_back(cv::DMatch(m.first, m.second, FLT_MAX));
  }
  return dm;
}


} // end namespace ocv

} // end namespace maptk
