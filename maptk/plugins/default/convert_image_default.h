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

#ifndef _MAPTK_PLUGINS_DEFAULT_CONVERT_IMAGE_DEFAULT_H_
#define _MAPTK_PLUGINS_DEFAULT_CONVERT_IMAGE_DEFAULT_H_

#include <maptk/algo/convert_image.h>

#include <maptk/plugins/default/plugin_default_config.h>


namespace maptk
{

namespace algo
{


/// A class for bypassing image conversion
class PLUGIN_DEFAULT_EXPORT convert_image_default
  : public algorithm_impl<convert_image_default, convert_image>
{
public:
   /// Default Constructor
  convert_image_default();

  /// Copy Constructor
  convert_image_default(const convert_image_default&);

  /// Return the name of this implementation
  virtual std::string impl_name() const { return "default"; }

  /// Default image converter ( does nothing )
  /**
   * \param [in] img image to be converted
   * \returns the input image
   */
  virtual image_container_sptr convert(image_container_sptr img) const;
};


} // end namespace algo

} // end namespace maptk


#endif // _MAPTK_PLUGINS_DEFAULT_CONVERT_IMAGE_DEFAULT_H_
