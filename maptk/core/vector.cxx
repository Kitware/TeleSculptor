/*ckwg +29
 * Copyright 2013-2014 by Kitware, Inc.
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
 * \brief Implementation of \link maptk::vector_ vector_<N,T> \endlink and
 *        related classes for \c T = { \c float, \c double } and
 *        \c N = { 2, 3, 4}
 */

#include "vector.h"


namespace maptk
{


/// output stream operator for a vector
template <unsigned N, typename T>
std::ostream&  operator<<(std::ostream& s, const vector_<N,T>& v)
{
  s << v[0];
  for( unsigned i=1; i<N; ++i )
  {
    s << " " << v[i];
  }
  return s;
}

/// input stream operator for a vector
template <unsigned N, typename T>
std::istream&  operator>>(std::istream& s, vector_<N,T>& v)
{
  for( unsigned i=0; i<N; ++i)
  {
    s >> std::skipws >> v[i];
  }
  return s;
}


/// \cond DoxygenSuppress
#define INSTANTIATE_VECTOR(N,T) \
template class MAPTK_CORE_EXPORT vector_<N,T>; \
template MAPTK_CORE_EXPORT std::ostream&  operator<<(std::ostream& s, const vector_<N,T>& v); \
template MAPTK_CORE_EXPORT std::istream&  operator>>(std::istream& s, vector_<N,T>& v)

INSTANTIATE_VECTOR(2, double);
INSTANTIATE_VECTOR(2, float);
INSTANTIATE_VECTOR(3, double);
INSTANTIATE_VECTOR(3, float);
INSTANTIATE_VECTOR(4, double);
INSTANTIATE_VECTOR(4, float);

#undef INSTANTIATE_VECTOR
/// \endcond

} // end namespace maptk
