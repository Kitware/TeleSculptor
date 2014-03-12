/*ckwg +5
 * Copyright 2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

/**
 * \file
 * \brief Implementation for IO exceptions
 */

#include "math.h"

namespace maptk
{


math_exception
::math_exception() MAPTK_NOTHROW
{
  m_what = "A math exception occurred.";
}

math_exception
::~math_exception() MAPTK_NOTHROW
{
}


non_invertible_matrix
::non_invertible_matrix() MAPTK_NOTHROW
{
  m_what = "A matrix was found to be non-invertible";
}

non_invertible_matrix
::~non_invertible_matrix() MAPTK_NOTHROW
{
}


point_maps_to_infinity
::point_maps_to_infinity() MAPTK_NOTHROW
{
  m_what = "A point mapped to infinity";
}

point_maps_to_infinity
::~point_maps_to_infinity() MAPTK_NOTHROW
{
}


} // end maptk namespace
