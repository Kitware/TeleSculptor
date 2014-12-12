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
 * \brief VisCL algorithm registration function implementation
 */

#include "register_algorithms.h"

#include <maptk/logging_macros.h>
#include <maptk/plugins/viscl/convert_image.h>
#include <maptk/plugins/viscl/detect_features.h>
#include <maptk/plugins/viscl/extract_descriptors.h>
#include <maptk/plugins/viscl/match_features.h>
#include <maptk/registrar.h>


namespace maptk
{

namespace vcl
{

/// Register VisCL algorithm implementations with the given or global registrar
int register_algorithms( maptk::registrar &reg )
{
  try
  {
    int expected = 4,
        registered
      = maptk::vcl::convert_image::register_self( reg )
      + maptk::vcl::detect_features::register_self( reg )
      + maptk::vcl::extract_descriptors::register_self( reg )
      + maptk::vcl::match_features::register_self( reg )
      ;
    LOG_DEBUG( "plugins::viscl::register_algorithms",
               "Registered " << registered << " of " << expected << " algorithms" );
    return expected - registered;
  }
  catch (...)
  {
    LOG_ERROR( "plugins::viscl::register_algo_impls",
               "Exception caught during algorithm registration" );
  }
  return -1;
}

} // end vcl ns

} // end maptk ns
