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
 * \brief OCV BRIEF descriptor extractor wrapper
 */

#ifndef MAPTK_HAS_OPENCV_VER_3

#ifndef MAPTK_EXTRACT_DESCRIPTORS_BRIEF_H_
#define MAPTK_EXTRACT_DESCRIPTORS_BRIEF_H_

#include <memory>
#include <string>

#include <vital/algo/extract_descriptors.h>
#include <vital/vital_config.h>

#include <maptk/plugins/ocv/extract_descriptors.h>

namespace kwiver {
namespace maptk {
namespace ocv {

class MAPTK_OCV_EXPORT extract_descriptors_BRIEF
  : public kwiver::vital::algorithm_impl<extract_descriptors_BRIEF,
                                         extract_descriptors,
                                         vital::algo::extract_descriptors>
{
public:
  /// Constructor
  extract_descriptors_BRIEF();

  /// Copy Constructor
  /**
   * \param other The other BRIEF descriptor extractor to copy
   */
  extract_descriptors_BRIEF(extract_descriptors_BRIEF const &other);

  /// Destructor
  virtual ~extract_descriptors_BRIEF();

  /// Return the name of this implementation
  virtual std::string impl_name() const { return "ocv_BRIEF"; }
  /// Returns a descriptive string for this implementation
  virtual std::string description() const {
    return "OpenCV feature-point descriptor extraction via the BRIEF algorithm";
  }

  /// Get this algorithm's \link kwiver::vital::config_block configuration block \endlink
  virtual vital::config_block_sptr get_configuration() const;
  /// Set this algorithm's properties via a config block
  virtual void set_configuration(vital::config_block_sptr config);
  /// Check that the algorithm's configuration config_block is valid
  virtual bool check_configuration(vital::config_block_sptr config) const;

private:
  /// private implementation class
  class priv;
  std::unique_ptr<priv> const p_;
};


} // end namespace ocv
} // end namespace maptk
} // end namespace kwiver

#endif //MAPTK_EXTRACT_DESCRIPTORS_BRIEF_H_

#endif
