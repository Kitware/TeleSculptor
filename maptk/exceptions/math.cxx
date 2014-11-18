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


invalid_matrix_operation
::invalid_matrix_operation(std::string reason) MAPTK_NOTHROW
{
  m_what = "Invalid operation: " + reason;
}

invalid_matrix_operation
::~invalid_matrix_operation() MAPTK_NOTHROW
{
}


} // end maptk namespace
