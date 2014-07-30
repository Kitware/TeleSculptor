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

#ifndef MAPTK_VISCL_CONVERT_IMAGE_H_
#define MAPTK_VISCL_CONVERT_IMAGE_H_


#include <maptk/core/algo/convert_image.h>

#include "viscl_config.h"

namespace maptk
{

namespace vcl
{

/// Class to convert an image to a viscl base image
class MAPTK_VISCL_EXPORT convert_image
  : public algo::algorithm_impl<convert_image, algo::convert_image>
{
public:

  /// Default Constructor
  convert_image();

  /// Copy Constructor
  convert_image(const convert_image &);

  /// Return the name of this implementation
  virtual std::string impl_name() const { return "viscl"; }

  /// Image convert to viscl underlying type
  /**
   * \param [in] img image to be converted
   * \returns the image container with underlying viscl img
   * should be used to prevent repeated image uploading to GPU
   */
  virtual image_container_sptr convert(image_container_sptr img) const;
};


} // end namespace vcl

} // end namespace maptk


#endif // MAPTK_VISCL_CONVERT_IMAGE_H_
