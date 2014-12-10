/*ckwg +29
 * Copyright 2011-2014 by Kitware, Inc.
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

#include <maptk/vector.h>
#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>

namespace maptk
{

namespace testing
{

/// random number generator type
typedef boost::mt19937 rng_t;
/// normal distribution
typedef boost::normal_distribution<> norm_dist_t;
/// normal distribution random generator type
typedef boost::variate_generator<rng_t&, norm_dist_t> normal_gen_t;

/// a global random number generator instance
static rng_t rng;


inline
maptk::vector_3d random_point3d(double stdev)
{
  normal_gen_t norm(rng, norm_dist_t(0.0, stdev));
  maptk::vector_3d v(norm(), norm(), norm());
  return v;
}


inline
maptk::vector_2d random_point2d(double stdev)
{
  normal_gen_t norm(rng, norm_dist_t(0.0, stdev));
  maptk::vector_2d v(norm(), norm());
  return v;
}

} // end namespace testing

} // end namespace maptk

#endif // MAPTK_TEST_TEST_RANDOM_POINT_H_
