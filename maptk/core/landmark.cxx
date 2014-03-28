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
 * \brief Implementation and template instantiation for
 * \link maptk::landmark landmark \endlink objects
 */

#include "landmark.h"
#include "transform.h"


namespace maptk
{


/// output stream operator for a landmark base class
std::ostream&  operator<<(std::ostream& s, const landmark& m)
{
  // TODO include covariance once stream operators are defined
  s << m.loc() << " "
    << m.scale();
  return s;
}


/// Default Constructor
template <typename T>
landmark_<T>
::landmark_()
: loc_(0,0,0),
  scale_(1)
{
}


/// Constructor for a feature
template <typename T>
landmark_<T>
::landmark_(const vector_3_<T>& loc, T scale)
: loc_(loc),
  scale_(scale)
{
}


/// Transform the landmark by applying a similarity transformation in place
template <typename T>
landmark_<T>&
landmark_<T>
::apply_transform(const similarity_<T>& xform)
{
  this->loc_ = xform * this->loc_;
  this->scale_ *= xform.scale();
  this->covar_ = maptk::transform(this->covar_, xform);
  return *this;
}


/// output stream operator for a landmark
template <typename T>
std::ostream&  operator<<(std::ostream& s, const landmark_<T>& m)
{
  // TODO include covariance once stream operators are defined
  s << m.get_loc() << " "
    << m.get_scale();
  return s;
}


/// input stream operator for a landmark
template <typename T>
std::istream&  operator>>(std::istream& s, landmark_<T>& m)
{
  // TODO include covariance once stream operators are defined
  vector_3_<T> loc;
  T scale;
  s >> loc
    >> scale;
  m.set_loc(loc);
  m.set_scale(scale);
  return s;
}


/// \cond DoxygenSuppress
#define INSTANTIATE_LANDMARK(T) \
template class MAPTK_CORE_EXPORT landmark_<T>; \
template MAPTK_CORE_EXPORT std::ostream& operator<<(std::ostream& s, const landmark_<T>& f); \
template MAPTK_CORE_EXPORT std::istream& operator>>(std::istream& s, landmark_<T>& f)

INSTANTIATE_LANDMARK(double);
INSTANTIATE_LANDMARK(float);

#undef INSTANTIATE_LANDMARK
/// \endcond

} // end namespace maptk
