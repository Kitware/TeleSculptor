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
 * \brief OCV match_features algorithm implementation
 */

#include "match_features.h"

#include <vector>

#include <vital/algo/algorithm.txx>  // for INSTANTIATE_ALGORITHM_DEF macro

#include <maptk/plugins/ocv/descriptor_set.h>
#include <maptk/plugins/ocv/match_set.h>


using namespace kwiver::vital;

namespace kwiver {
namespace maptk {
namespace ocv
{


/// Match one set of features and corresponding descriptors to another
vital::match_set_sptr
match_features
::match(vital::feature_set_sptr feat1, vital::descriptor_set_sptr desc1,
        vital::feature_set_sptr feat2, vital::descriptor_set_sptr desc2) const
{
  // Return empty match set pointer if either of the input sets were empty
  // pointers
  if( !desc1 || !desc2 )
  {
    return vital::match_set_sptr();
  }
  // Only perform matching if both pointers are valid and if both descriptor
  // sets contain non-zero elements
  if( !desc1->size() || !desc2->size() )
  {
    return vital::match_set_sptr( new maptk::ocv::match_set() );
  }

  cv::Mat d1 = descriptors_to_ocv_matrix(*desc1);
  cv::Mat d2 = descriptors_to_ocv_matrix(*desc2);
  std::vector<cv::DMatch> matches;
  ocv_match(d1, d2, matches);
  return vital::match_set_sptr(new maptk::ocv::match_set(matches));
}


} // end namespace ocv
} // end namespace maptk
} // end namespace kwiver


/// \cond DoygenSupress
INSTANTIATE_ALGORITHM_DEF( kwiver::maptk::ocv::match_features );
/// \endcond
