/*ckwg +29
 * Copyright 2014-2016 by Kitware, Inc.
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
 * \brief Helper utility functions for getting, setting and checking properties
 *        between OpenCV algorithm properties and vital::config_block objects.
 */

#ifndef MAPTK_PLUGINS_OCV_OCV_ALGO_TOOLS_H_
#define MAPTK_PLUGINS_OCV_OCV_ALGO_TOOLS_H_


#include <string>
#include <iostream>

#include <opencv2/core/core.hpp>

#include <vital/vital_config.h>
#include <vital/vital_types.h>
#include <vital/config/config_block.h>
#include <vital/exceptions.h>
#include <vital/logger/logger.h>

#include <maptk/plugins/ocv/maptk_ocv_export.h>


namespace kwiver {
namespace maptk {
namespace ocv {
namespace helper_ {

/// Templated helper method for creating a new OpenCV algorithm instance.
/**
 * \tparam algo_t cv::Algorithm class or sub-class to attempt creation from. We
 *                will always fall-back to attempting
 *                \c cv::Algorithm::create(impl_name) if creation with the
 *                specific sub-class fails.
 */
template <typename algo_t>
cv::Ptr<algo_t> create_ocv2_algo(std::string const& impl_name)
{
  kwiver::vital::logger_handle_t log =
      kwiver::vital::get_logger("maptk::ocv::helpers::create_ocv2_algo");
  cv::Ptr<algo_t> a;

#ifndef MAPTK_HAS_OPENCV_VER_3
  // attempt to use the given type to natively create the algorithm with
  // possible special rules contained in the subclass. If this does not
  // yield a valid instance, attempt creating an algorithm using the base
  // cv::Algorithm creation rules.
  try
  {
    a = algo_t::create(impl_name);
  }
  catch (cv::Exception const&)
  {
    log->log_info("Caught OpenCV creation error, attempting fall-back.");
  }

  // if the create call returned something empty or an error occurred, fall back
  // to trying the top-level cv::Algorithm constructor.
  if (a.empty())
  {
    a = cv::Algorithm::create<algo_t>(impl_name);
  }
  else if (!a->info())
  {
    throw vital::algorithm_exception("OpenCV", impl_name, "OCV failed to construct "
        "underlying algorithm info object of " + impl_name + " algorithm, "
        "returning an invalid algorithm object. Cannot proceed.");
  }
#else

#endif

  return a;
}


/// cv::Algorithm specialization when given type is cv::Algorithm
/**
 * Create call on a cv::Algorithm is performed differently than on sub-classes
 * (its templated itself vs. other not being so).
 */
template <>
MAPTK_OCV_EXPORT
cv::Ptr<cv::Algorithm> create_ocv2_algo<cv::Algorithm>(std::string const& impl_name)
{
  kwiver::vital::logger_handle_t log =
      kwiver::vital::get_logger("maptk::ocv::helpers::create_ocv2_algo");
  log->log_debug("Creating empty OpenCV 2.4.x algorithm instance");
  return cv::Algorithm::create<cv::Algorithm>(impl_name);
}


} // end namespace helper_
} // end namespace ocv
} // end namespace maptk
} // end namespace kwiver

#endif // MAPTK_PLUGINS_OCV_OCV_ALGO_TOOLS_H_
