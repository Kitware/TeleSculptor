/*ckwg +29
 * Copyright 2014 by Kitware, Inc.
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

#ifndef MAPTK_VISCL_DETECT_FEATURES_H_
#define MAPTK_VISCL_DETECT_FEATURES_H_

#include <maptk/core/algo/detect_features.h>
#include <maptk/viscl/viscl_config.h>
#include <boost/scoped_ptr.hpp>

namespace maptk
{

namespace vcl
{

/// An algorithm class for detecting feature points using VisCL
class MAPTK_VISCL_EXPORT detect_features
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
  virtual std::string impl_name() const { return "viscl"; }

  /// Get this algorithm's \link maptk::config_block configuration block \endlink
  virtual config_block_sptr get_configuration() const;

  /// Set this algorithm's properties via a config block
  virtual void set_configuration(config_block_sptr config);

  /// Check that the algorithm's configuration config_block is valid
  virtual bool check_configuration(config_block_sptr config) const;

  /// Extract a set of image features from the provided image
  /**
    * \param image_data contains the image data to process
    * \returns a set of image features
    */
  virtual feature_set_sptr
  detect(image_container_sptr image_data,
         image_container_sptr mask = image_container_sptr()) const;

private:
  /// private implementation class
  class priv;
  boost::scoped_ptr<priv> d_;
};

} // end namespace vcl

} // end namespace maptk


#endif // MAPTK_VISCL_DETECT_FEATURES_H_
