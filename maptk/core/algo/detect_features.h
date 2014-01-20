/*ckwg +5
 * Copyright 2013-2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#ifndef MAPTK_ALGO_DETECT_FEATURES_H_
#define MAPTK_ALGO_DETECT_FEATURES_H_

#include <boost/shared_ptr.hpp>

#include <maptk/core/algo/algorithm.h>
#include <maptk/core/feature_set.h>
#include <maptk/core/image_container.h>

namespace maptk
{

namespace algo
{

/// An abstract base class for detecting feature points
class MAPTK_CORE_EXPORT detect_features
  : public algorithm_def<detect_features>
{
public:
  /// Return the name of this algorithm
  std::string type_name() const { return "detect_features"; }

  /// Extract a set of image features from the provided image
  /// \param image_data contains the image data to process
  /// \returns a set of image features
  virtual feature_set_sptr
  detect(image_container_sptr image_data) const = 0;

};


typedef boost::shared_ptr<detect_features> detect_features_sptr;


} // end namespace algo

} // end namespace maptk


#endif // MAPTK_ALGO_DETECT_FEATURES_H_
