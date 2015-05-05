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
 * \brief VXL image_io implementation
 */

#include "image_io.h"

#include <maptk/logging_macros.h>
#include <maptk/plugins/vxl/image_container.h>
#include <maptk/eigen_io.h>
#include <maptk/vector.h>

#include <vil/vil_convert.h>
#include <vil/vil_load.h>
#include <vil/vil_save.h>


namespace maptk
{

namespace vxl
{

/// Private implementation class
class image_io::priv
{
public:
  /// Constructor
  priv()
  : intensity_range(0, 0)
  {
  }

  priv(const priv& other)
  : intensity_range(other.intensity_range)
  {
  }

  vector_2d intensity_range;
};


/// Constructor
image_io
::image_io()
: d_(new priv)
{
}


/// Copy Constructor
image_io
::image_io(const image_io& other)
: d_(new priv(*other.d_))
{
}


/// Destructor
image_io
::~image_io()
{
}



/// Get this algorithm's \link maptk::config_block configuration block \endlink
config_block_sptr
image_io
::get_configuration() const
{
  // get base config from base class
  config_block_sptr config = maptk::algo::image_io::get_configuration();
  config->set_value("intensity_range", d_->intensity_range,
                    "The range of intensity values (min, max) to stretch into "
                    "the byte range.  This is most useful when e.g. 12-bit "
                    "data is encoded in 16-bit pixels");
  return config;
}


/// Set this algorithm's properties via a config block
void
image_io
::set_configuration(config_block_sptr in_config)
{
  // Starting with our generated config_block to ensure that assumed values are present
  // An alternative is to check for key presence before performing a get_value() call.
  config_block_sptr config = this->get_configuration();
  config->merge_config(in_config);

  d_->intensity_range = config->get_value<vector_2d>("intensity_range",
                                        d_->intensity_range.transpose());
}


/// Check that the algorithm's currently configuration is valid
bool
image_io
::check_configuration(config_block_sptr config) const
{
  return true;
}


/// Load image image from the file
image_container_sptr
image_io
::load_(const std::string& filename) const
{
  LOG_DEBUG( "maptk::vxl::image_io::load",
             "Loading image from file: " << filename );

  vil_image_resource_sptr img_rsc = vil_load_image_resource(filename.c_str());
  vil_image_view<vxl_byte> img;
  switch (img_rsc->pixel_format())
  {
  case VIL_PIXEL_FORMAT_BOOL:
    // If a boolean image, true-value pixls are represented in the
    // vxl_byte image as 1's.
    img = vil_convert_cast(vxl_byte(), img_rsc->get_view());
    break;
  case VIL_PIXEL_FORMAT_UINT_16:
    {
      vil_image_view<vxl_uint_16> img16 = img_rsc->get_view();
      vxl_uint_16 minv = d_->intensity_range[0], maxv = d_->intensity_range[1];
      vil_convert_stretch_range_limited(img16, img, minv, maxv);
    }
    break;
  default:
    img = img_rsc->get_view();
  }
  return image_container_sptr(new vxl::image_container(img));

  LOG_DEBUG( "maptk::vxl::image_io::load",
             "Image stats (" << filename << "):" << std::endl <<
             "\tni: " << img.ni() << std::endl <<
             "\tnj: " << img.nj() << std::endl <<
             "\tnplanes: " << img.nplanes() << std::endl <<
             "\tsize: " << img.size()
             );
  return image_container_sptr(new vxl::image_container(img));
}


/// Save image image to a file
void
image_io
::save_(const std::string& filename,
       image_container_sptr data) const
{
  vil_save(vxl::image_container::maptk_to_vxl(data->get_image()),
           filename.c_str());
}

} // end namespace vxl

} // end namespace maptk
