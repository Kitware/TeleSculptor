/*ckwg +5
 * Copyright 2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */


#include <maptk/viscl/feature_set.h>

namespace maptk
{

namespace viscl
{


/// Return a vector of feature shared pointers
std::vector<feature_sptr>
feature_set
::features() const
{
  std::vector<feature_sptr> features;
  // TODO download the data and construct VisCL features
  return features;
}


/// Convert any feature set to a VisCL data (upload if needed)
//type
//features_to_viscl(const maptk::feature_set& features);
//{
//  //if already on GPU in VisCL format, then access it
//  if( const viscl::feature_set* f =
//          dynamic_cast<const viscl::feature_set*>(&feat_set) )
//  {
//    return f->viscl_features();
//  }
//  //TODO otherwise convert/upload the features
//  return viscl_features;
//}


} // end namespace viscl

} // end namespace maptk
