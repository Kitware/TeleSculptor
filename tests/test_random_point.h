/*ckwg +5
 * Copyright 2011-2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

/**
 * \file
 *
 * \brief Functions for creating test points with added random Gaussian noise.
 *
 */

#ifndef MAPTK_TEST_TEST_RANDOM_POINT_H_
#define MAPTK_TEST_TEST_RANDOM_POINT_H_

#include <maptk/core/vector.h>
#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>


/// random number generator type
typedef boost::mt19937 rng_t;
/// normal distribution
typedef boost::normal_distribution<> norm_dist_t;
/// normal distribution random generator type
typedef boost::variate_generator<rng_t&, norm_dist_t> normal_gen_t;

/// a global random number generator instance
static rng_t rng;


inline
maptk::vector_3d random_point3(double stdev)
{
  normal_gen_t norm(rng, norm_dist_t(0.0, stdev));
  maptk::vector_3d v(norm(), norm(), norm());
  return v;
}


inline
maptk::vector_2d random_point2(double stdev)
{
  normal_gen_t norm(rng, norm_dist_t(0.0, stdev));
  maptk::vector_2d v(norm(), norm());
  return v;
}


#endif // MAPTK_TEST_TEST_RANDOM_POINT_H_
