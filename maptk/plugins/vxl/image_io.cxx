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

#include <kwiver_util/logger/logger.h>
#include <maptk/plugins/vxl/image_container.h>
#include <vital/eigen_io.h>
#include <vital/vector.h>
#include <vital/exceptions/image.h>

#include <vil/vil_convert.h>
#include <vil/vil_load.h>
#include <vil/vil_save.h>

using namespace kwiver::vital;

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
  : auto_stretch(false),
    manual_stretch(false),
    intensity_range(0, 255),
    m_logger( kwiver::get_logger( "maptk_vxl_image_io" ) )
  {
  }

  priv(const priv& other)
  : auto_stretch(other.auto_stretch),
    manual_stretch(other.manual_stretch),
    intensity_range(other.intensity_range)
  {
  }

  bool auto_stretch;
  bool manual_stretch;
  vector_2d intensity_range;

  kwiver::logger_handle_t m_logger;
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



/// Get this algorithm's \link maptk::kwiver::config_block configuration block \endlink
kwiver::config_block_sptr
image_io
::get_configuration() const
{
  // get base config from base class
  kwiver::config_block_sptr config = maptk::algo::image_io::get_configuration();

  config->set_value("auto_stretch", d_->auto_stretch,
                    "Dynamically stretch the range of the input data such that "
                    "the minimum and maximum pixel values in the data map to "
                    "0 and 255 in the byte image.  Warning, this can result in "
                    "brightness and constrast varying between images.");
  config->set_value("manual_stretch", d_->manual_stretch,
                    "Manually stretch the range of the input data by "
                    "specifying the minimum and maximum values of the data "
                    "to map to the full byte range");
  if( d_->manual_stretch )
  {
    config->set_value("intensity_range", d_->intensity_range.transpose(),
                      "The range of intensity values (min, max) to stretch into "
                      "the byte range.  This is most useful when e.g. 12-bit "
                      "data is encoded in 16-bit pixels");
  }
  return config;
}


/// Set this algorithm's properties via a config block
void
image_io
::set_configuration(kwiver::config_block_sptr in_config)
{
  // Starting with our generated kwiver::config_block to ensure that assumed values are present
  // An alternative is to check for key presence before performing a get_value() call.
  kwiver::config_block_sptr config = this->get_configuration();
  config->merge_config(in_config);

  d_->auto_stretch = config->get_value<bool>("auto_stretch",
                                              d_->auto_stretch);
  d_->manual_stretch = config->get_value<bool>("manual_stretch",
                                              d_->manual_stretch);
  d_->intensity_range = config->get_value<vector_2d>("intensity_range",
                                        d_->intensity_range.transpose());
}


/// Check that the algorithm's currently configuration is valid
bool
image_io
::check_configuration(kwiver::config_block_sptr config) const
{
  double auto_stretch = config->get_value<bool>("auto_stretch",
                                                d_->auto_stretch);
  double manual_stretch = config->get_value<bool>("manual_stretch",
                                                  d_->manual_stretch);
  if( auto_stretch && manual_stretch)
  {
    LOG_ERROR( d_->m_logger, "can not enable both manual and auto stretching");
    return false;
  }
  if( manual_stretch )
  {
    vector_2d range = config->get_value<vector_2d>("intensity_range",
                                        d_->intensity_range.transpose());
    if( range[0] >= range[1] )
    {
      LOG_ERROR( d_->m_logger, "stretching range minimum not less than maximum"
                <<" ("<<range[0]<<", "<<range[1]<<")");
      return false;
    }
  }
  return true;
}


/// Load image image from the file
image_container_sptr
image_io
::load_(const std::string& filename) const
{
  LOG_DEBUG( d_->m_logger, "Loading image from file: " << filename );

  vil_image_resource_sptr img_rsc = vil_load_image_resource(filename.c_str());
  vil_image_view<vxl_byte> img;

#define DO_CASE(T)                                                     \
  case T:                                                              \
    {                                                                  \
      typedef vil_pixel_format_type_of<T >::component_type pix_t;      \
      vil_image_view<pix_t> img_pix_t = img_rsc->get_view();           \
      if( d_->auto_stretch )                                           \
      {                                                                \
        vil_convert_stretch_range(img_pix_t, img);                     \
      }                                                                \
      else if( d_->manual_stretch )                                    \
      {                                                                \
        pix_t minv = static_cast<pix_t>(d_->intensity_range[0]);       \
        pix_t maxv = static_cast<pix_t>(d_->intensity_range[1]);       \
        vil_convert_stretch_range_limited(img_pix_t, img, minv, maxv); \
      }                                                                \
      else                                                             \
      {                                                                \
        vil_convert_cast(img_pix_t, img);                              \
      }                                                                \
    }                                                                  \
    break;                                                             \

  switch (img_rsc->pixel_format())
  {
    // special case for bool because manual stretching limits do not
    // make sense and trigger compiler warnings on some platforms.
    case VIL_PIXEL_FORMAT_BOOL:
      {
        vil_image_view<bool> img_bool = img_rsc->get_view();
        if( d_->auto_stretch || d_->manual_stretch )
        {
          vil_convert_stretch_range(img_bool, img);
        }
        else
        {
          vil_convert_cast(img_bool, img);
        }
      }
      break;
    DO_CASE(VIL_PIXEL_FORMAT_BYTE);
    DO_CASE(VIL_PIXEL_FORMAT_SBYTE);
    DO_CASE(VIL_PIXEL_FORMAT_UINT_16);
    DO_CASE(VIL_PIXEL_FORMAT_INT_16);
    DO_CASE(VIL_PIXEL_FORMAT_UINT_32);
    DO_CASE(VIL_PIXEL_FORMAT_INT_32);
    DO_CASE(VIL_PIXEL_FORMAT_UINT_64);
    DO_CASE(VIL_PIXEL_FORMAT_INT_64);
    DO_CASE(VIL_PIXEL_FORMAT_FLOAT);
    DO_CASE(VIL_PIXEL_FORMAT_DOUBLE);

  default:
    if( d_->auto_stretch )
    {
      // automatically stretch to fill the byte range using the
      // minimum and maximum pixel values
      img = vil_convert_stretch_range(vxl_byte(), img_rsc->get_view());
    }
    else if( d_->manual_stretch )
    {
      LOG_ERROR( d_->m_logger, "Unable to manually stretch pixel type: "
                << img_rsc->pixel_format());
      throw image_exception();
    }
    else
    {
      img = vil_convert_cast(vxl_byte(), img_rsc->get_view());
    }
  }
#undef DO_CASE
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
