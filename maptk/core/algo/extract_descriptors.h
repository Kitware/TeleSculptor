/*ckwg +5
 * Copyright 2013-2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

/**
 * \file
 * \brief extract_descriptors algorithm definition
 */

#ifndef MAPTK_ALGO_EXTRACT_DESCRIPTORS_H_
#define MAPTK_ALGO_EXTRACT_DESCRIPTORS_H_

#include <maptk/core/algo/algorithm.h>
#include <maptk/core/image_container.h>
#include <maptk/core/feature_set.h>
#include <maptk/core/descriptor_set.h>
#include <boost/shared_ptr.hpp>

namespace maptk
{

namespace algo
{

/// An abstract base class for extracting feature descriptors
class MAPTK_CORE_EXPORT extract_descriptors
  : public algorithm_def<extract_descriptors>
{
public:
  /// Return the name of this algorithm
  std::string type_name() const { return "extract_descriptors"; }

  /// Extract from the image a descriptor corresoponding to each feature
  /**
   * \param image_data contains the image data to process
   * \param features the feature locations at which descriptors are extracted
   * \returns a set of feature descriptors
   */
  virtual descriptor_set_sptr
  extract(image_container_sptr image_data,
          feature_set_sptr features) const = 0;

};


/// Shared pointer for base extract_descriptors algorithm definition class
typedef boost::shared_ptr<extract_descriptors> extract_descriptors_sptr;


} // end namespace algo

} // end namespace maptk


#endif // MAPTK_ALGO_EXTRACT_DESCRIPTORS_H_
