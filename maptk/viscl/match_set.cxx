/*ckwg +5
 * Copyright 2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */


#include <maptk/viscl/match_set.h>
#include <boost/foreach.hpp>

namespace maptk
{

namespace viscl
{

/// Return a vector of matching indices
std::vector<match>
match_set
::matches() const
{
  std::vector<match> m;
  // TODO download matches and convert
  return m;
}


/// Convert any match set to VisCL matches
//TODO define 'type'
//type
//matches_to_viscl(const maptk::match_set& m_set)
//{
//  if( const viscl::match_set* m_viscl =
//          dynamic_cast<const viscl::match_set*>(&m_set) )
//  {
//    return m_viscl->viscl_matches();
//  }
//  const std::vector<match> mats = m_set.matches();
//  //TODO convert and upload matches to GPU
//  return viscl_data;
//}


} // end namespace viscl

} // end namespace maptk
