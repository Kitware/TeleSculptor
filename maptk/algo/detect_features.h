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
 * \brief detect_features algorithm definition
 */

#ifndef MAPTK_ALGO_DETECT_FEATURES_H_
#define MAPTK_ALGO_DETECT_FEATURES_H_

#include <maptk/config.h>

#include <boost/shared_ptr.hpp>

#include <vital/algorithm.h>
#include <vital/feature_set.h>
#include <vital/image_container.h>

namespace maptk
{

namespace algo
{

/// An abstract base class for detecting feature points
class MAPTK_LIB_EXPORT detect_features
  : public kwiver::vital::algorithm_def<detect_features>
{
public:
  /// Return the name of this algorithm
  static std::string static_type_name() { return "detect_features"; }

  /// Extract a set of image features from the provided image
  /**
   * A given mask image should be one-channel (mask->depth() == 1). If the
   * given mask image has more than one channel, only the first will be
   * considered.
   *
   * \throws image_size_mismatch_exception
   *    When the given non-zero mask image does not match the size of the
   *    dimensions of the given image data.
   *
   * \param image_data contains the image data to process
   * \param mask Mask image where regions of positive values (boolean true)
   *             indicate regions to consider. Only the first channel will be
   *             considered.
   * \returns a set of image features
   */
  virtual kwiver::vital::feature_set_sptr
  detect(kwiver::vital::image_container_sptr image_data,
         kwiver::vital::image_container_sptr mask = kwiver::vital::image_container_sptr()) const = 0;

};


/// Shared pointer for detect_features algorithm definition class
typedef boost::shared_ptr<detect_features> detect_features_sptr;


} // end namespace algo

} // end namespace maptk


#endif // MAPTK_ALGO_DETECT_FEATURES_H_
