/*ckwg +29
 * Copyright 2016 by Kitware, Inc.
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
 * \brief OCV GFTT feature detector wrapper
 */

#ifndef MAPTK_DETECT_FEATURES_GFTT_H_
#define MAPTK_DETECT_FEATURES_GFTT_H_

#include <memory>
#include <string>

#include <vital/vital_config.h>

#include <maptk/plugins/ocv/detect_features.h>
#include <maptk/plugins/ocv/maptk_ocv_export.h>

namespace kwiver {
namespace maptk {
namespace ocv {


class MAPTK_OCV_EXPORT detect_features_GFTT
  : public vital::algorithm_impl< detect_features_GFTT,
                                  ocv::detect_features,
                                  vital::algo::detect_features >
{
public:
  /// Constructor
  detect_features_GFTT();

  /// Copy constructor
  detect_features_GFTT(const detect_features_GFTT &other);

  /// Destructor
  virtual ~detect_features_GFTT();

  /// Return the name of this implementation
  virtual std::string impl_name() const { return "ocv_GFTT"; }
  /// Returns a descriptive string for this implementation
  virtual std::string description() const {
    return "OpenCV feature detection via the GFTT algorithm";
  }

  /// Get this algorithm's \link maptk::kwiver::config_block configuration block \endlink
  virtual vital::config_block_sptr get_configuration() const;
  /// Set this algorithm's properties via a config block
  virtual void set_configuration(vital::config_block_sptr config);
  /// Check that the algorithm's configuration vital::config_block is valid
  virtual bool check_configuration(vital::config_block_sptr config) const;

private:
  class priv;
  std::unique_ptr<priv> const p_;
};


} // end namespace kwiver
} // end namespace maptk
} // end namespace ocv

#endif //MAPTK_DETECT_FEATURES_GFTT_H_
