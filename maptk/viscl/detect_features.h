/*ckwg +5
 * Copyright 2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#ifndef MAPTK_VISCL_DETECT_FEATURES_H_
#define MAPTK_VISCL_DETECT_FEATURES_H_

#include <maptk/core/algo/detect_features.h>
#include <boost/scoped_ptr.hpp>

namespace maptk
{

namespace vcl
{

/// An algorithm class for detecting feature points using VisCL
class detect_features
: public algo::algorithm_impl<detect_features, algo::detect_features>
{
public:
  /// Constructor
  detect_features();

  /// Destructor
  virtual ~detect_features();

  /// Copy Constructor
  detect_features(const detect_features& other);

  /// Return the name of this implementation
  std::string impl_name() const { return "viscl"; }

  // No configuration yet for this class
  virtual void set_configuration(config_block_sptr /*config*/) { }
  virtual bool check_configuration(config_block_sptr /*config*/) const { return true; }

  /// Extract a set of image features from the provided image
  /// \param image_data contains the image data to process
  /// \returns a set of image features
  virtual feature_set_sptr
  detect(image_container_sptr image_data) const;

private:
  /// private implementation class
  class priv;
  boost::scoped_ptr<priv> d_;
};

} // end namespace viscl

} // end namespace maptk


#endif // MAPTK_VISCL_DETECT_FEATURES_H_
