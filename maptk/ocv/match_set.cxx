/*ckwg +5
 * Copyright 2013 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */


#include <maptk/ocv/match_set.h>
#include <boost/foreach.hpp>

namespace maptk
{

namespace ocv
{

/// Return a vector of matching indices
std::vector<match>
match_set
::matches() const
{
  std::vector<match> m;
  BOOST_FOREACH(cv::DMatch dm, this->data_)
  {
    m.push_back(match(dm.queryIdx, dm.trainIdx));
  }
  return m;
}


/// Convert any match set to a vector of OpenCV cv::DMatch
std::vector<cv::DMatch>
matches_to_ocv_dmatch(const maptk::match_set& m_set)
{
  if( const ocv::match_set* m_ocv =
          dynamic_cast<const ocv::match_set*>(&m_set) )
  {
    return m_ocv->ocv_matches();
  }
  std::vector<cv::DMatch> dm;
  const std::vector<match> mats = m_set.matches();
  BOOST_FOREACH(match m, mats)
  {
    dm.push_back(cv::DMatch(m.first, m.second, FLT_MAX));
  }
  return dm;
}


} // end namespace ocv

} // end namespace maptk
