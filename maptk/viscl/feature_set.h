/*ckwg +5
 * Copyright 2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#ifndef MAPTK_VISCL_FEATURE_SET_H_
#define MAPTK_VISCL_FEATURE_SET_H_


#include <maptk/core/feature_set.h>
#include <viscl/core/buffer.h>
#include <viscl/core/image.h>

namespace maptk
{

namespace vcl
{

/// A concrete feature set that wraps VisCL features
/// A VisCL feature only has the location set
/// It is possible to get the smoothing scale but that value is not
/// saved on the GPU so would have to be provided externally
class feature_set
: public maptk::feature_set
{
public:

  struct type
  {
    viscl::buffer features_;
    viscl::buffer numfeat_;
    viscl::image kptmap_;
  };

  /// Default Constructor
  feature_set() {}

  /// Constructor from VisCL data
  explicit feature_set(const type& viscl_features)
  : data_(viscl_features) {}

  /// Return the number of feature in the set
  /// The base version is fairly expensive
  virtual size_t size() const;

  /// Return a vector of feature shared pointers
  virtual std::vector<feature_sptr> features() const;

  /// Return the underlying VisCL features data structure
  const type& viscl_features() const { return data_; }

protected:

  /// The VisCL feature point data
  type data_;
};


/// Convert any feature set to a VisCL data (upload if needed)
/// width and height are the dimensions of the image that the features were
/// computed from - they are nessessary to create a search map for viscl tracker
/// viscl only cares about integer feature location, therefore you will lose info converting from
/// maptk feature set to viscl and back
feature_set::type
features_to_viscl(const maptk::feature_set& features, size_t width, size_t height);


} // end namespace viscl

} // end namespace maptk


#endif // MAPTK_VISCL_FEATURE_SET_H_
