/*ckwg +5
 * Copyright 2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#ifndef MAPTK_VISCL_FEATURE_SET_H_
#define MAPTK_VISCL_FEATURE_SET_H_


#include <maptk/core/feature_set.h>

namespace maptk
{

namespace viscl
{


/// A concrete feature set that wraps VisCL features
class feature_set
: public maptk::feature_set
{
public:
  /// Default Constructor
  feature_set() {}

  /// Constructor from VisCL data
  // TODO implement this
  //explicit feature_set(const type& viscl_features)
  //: data_(viscl_features) {}

  /// Return the number of feature in the set
  virtual size_t size() const { return 0; } // TODO return number of features

  /// Return a vector of feature shared pointers
  virtual std::vector<feature_sptr> features() const;

  /// Return the underlying VisCL features data structure
  // TODO implement this
  //const type& viscl_features() const { return data_; }

protected:

  /// The VisCL feature point data
  // TODO implement this
  //type data_;
};


/// Convert any feature set to a VisCL data (upload if needed)
// TODO implement this
//type
//features_to_viscl(const maptk::feature_set& features);


} // end namespace viscl

} // end namespace maptk


#endif // MAPTK_VISCL_FEATURE_SET_H_
