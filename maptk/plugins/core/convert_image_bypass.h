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

/**
 * \file
 * \brief Default convert_image algorithm that acts as a bypass
 */

#ifndef MAPTK_PLUGINS_CORE_CONVERT_IMAGE_BYPASS_H_
#define MAPTK_PLUGINS_CORE_CONVERT_IMAGE_BYPASS_H_

#include <maptk/algo/convert_image.h>

#include <maptk/plugins/core/plugin_core_config.h>


namespace maptk
{

namespace core
{


/// A class for bypassing image conversion
class PLUGIN_CORE_EXPORT convert_image_bypass
  : public algo::algorithm_impl<convert_image_bypass, algo::convert_image>
{
public:
   /// Default Constructor
  convert_image_bypass();

  /// Copy Constructor
  convert_image_bypass(const convert_image_bypass&);

  /// Return the name of this implementation
  virtual std::string impl_name() const { return "bypass"; }

  /// Return descriptive string for this implementation
  virtual std::string description() const;

  /// Default image converter ( does nothing )
  /**
   * \param [in] img image to be converted
   * \returns the input image
   */
  virtual image_container_sptr convert(image_container_sptr img) const;
};


} // end namespace core

} // end namespace maptk


#endif // MAPTK_PLUGINS_CORE_CONVERT_IMAGE_BYPASS_H_
