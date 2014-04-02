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

#ifndef MAPTK_ALGO_CONVERT_IMAGE_H_
#define MAPTK_ALGO_CONVERT_IMAGE_H_

#include <string>

#include <boost/shared_ptr.hpp>

#include <maptk/core/algo/algorithm.h>
#include <maptk/core/image_container.h>

#include <maptk/core/core_config.h>

namespace maptk
{

namespace algo
{

/// An abstract base class for converting base image type
class MAPTK_CORE_EXPORT convert_image
  : public algorithm_def<convert_image>
{
public:
  /// Return the name of this algorithm
  std::string type_name() const { return "convert_image"; }

  /// Set this algorithm's properties via a config block
  virtual void set_configuration(config_block_sptr config);
  /// Check that the algorithm's currently configuration is valid
  virtual bool check_configuration(config_block_sptr config) const;

  /// Convert image base type
  virtual image_container_sptr convert(image_container_sptr img) const = 0;

};

typedef boost::shared_ptr<convert_image> convert_image_sptr;

/// A class for bypassing image conversion
class MAPTK_CORE_EXPORT convert_image_default
  : public algorithm_impl<convert_image_default, convert_image>
{
public:
   /// Default Constructor
  convert_image_default();

  /// Copy Constructor
  convert_image_default(const convert_image_default&);

  /// Return the name of this implementation
  std::string impl_name() const { return "default"; }

  /// Default image converter ( does nothing )
  /**
   * \param [in] img image to be converted
   * \returns the input image
   */
  virtual image_container_sptr convert(image_container_sptr img) const;
};


} // end namespace algo

} // end namespace maptk


#endif // MAPTK_ALGO_CONVERT_IMAGE_H_
