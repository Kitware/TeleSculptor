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
 * \brief VXL image_io interface
 */

#ifndef MAPTK_PLUGINS_VXL_IMAGE_IO_H_
#define MAPTK_PLUGINS_VXL_IMAGE_IO_H_

#include <vital/algo/image_io.h>
#include <maptk/plugins/vxl/vxl_config.h>

#include <boost/scoped_ptr.hpp>

namespace kwiver {
namespace maptk {

namespace vxl
{

/// A class for using VXL to read and write images
class MAPTK_VXL_EXPORT image_io
  : public vital::algorithm_impl<image_io, vital::algo::image_io>
{
public:
  /// Constructor
  image_io();

  /// Destructor
  virtual ~image_io();

  /// Copy Constructor
  image_io(const image_io& other);

  /// Return the name of this implementation
  virtual std::string impl_name() const { return "vxl"; }

  /// Get this algorithm's \link vital::config_block configuration block \endlink
  virtual vital::config_block_sptr get_configuration() const;
  /// Set this algorithm's properties via a config block
  virtual void set_configuration(vital::config_block_sptr config);
  /// Check that the algorithm's currently configuration is valid
  virtual bool check_configuration(vital::config_block_sptr config) const;

private:
  /// Implementation specific load functionality.
  /*
   * NOTE: When loading boolean images (ppm, pbm, etc.), true-value regions are
   * represented in the returned image as regions of 1's.
   *
   * \param filename the path to the file the load
   * \returns an image container refering to the loaded image
   */
  virtual vital::image_container_sptr load_(const std::string& filename) const;

  /// Implementation specific save functionality.
  /**
   * \param filename the path to the file to save
   * \param data the image container refering to the image to write
   */
  virtual void save_(const std::string& filename,
                     vital::image_container_sptr data) const;

  /// private implementation class
  class priv;
  boost::scoped_ptr<priv> d_;
};

} // end namespace vxl

} // end namespace maptk
} // end namespace kwiver


#endif // MAPTK_PLUGINS_VXL_IMAGE_IO_H_
