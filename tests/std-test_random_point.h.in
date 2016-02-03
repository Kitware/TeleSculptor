/*ckwg +29
 * Copyright 2011-2016 by Kitware, Inc.
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
 *
 * \brief Functions for creating test points with added random Gaussian noise.
 *
 */

#ifndef MAPTK_TEST_TEST_RANDOM_POINT_H_
#define MAPTK_TEST_TEST_RANDOM_POINT_H_

#include <vital/types/vector.h>
#include <random>

namespace kwiver {
namespace maptk {

namespace testing
{

/// random number generator type
typedef std::mt19937 rng_t;

/// normal distribution
typedef std::normal_distribution<> norm_dist_t;

/// a global random number generator instance
static rng_t rng;


inline
kwiver::vital::vector_3d
random_point3d( double stdev )
{
  norm_dist_t norm( 0.0, stdev );
  kwiver::vital::vector_3d v( norm( rng ), norm( rng ), norm( rng ) );

  return v;
}


inline
kwiver::vital::vector_2d
random_point2d( double stdev )
{
  norm_dist_t norm( 0.0, stdev );
  kwiver::vital::vector_2d v( norm( rng ), norm( rng ) );

  return v;
}


} // end namespace testing

} // end namespace maptk
} // end namespace kwiver

#endif // MAPTK_TEST_TEST_RANDOM_POINT_H_
