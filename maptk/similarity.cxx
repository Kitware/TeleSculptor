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
 * \brief Implementation of \link maptk::similarity_ similarity_<T> \endlink
 *        for \c T = { \c float, \c double }
 */

#include "similarity.h"
#include <cmath>


namespace maptk
{

/// Constructor - from a matrix
template <typename T>
similarity_<T>
::similarity_(const Eigen::Matrix<T,4,4>& M)
{
  if (M(3,0) != T(0) || M(3,1) != T(0) || M(3,2) != T(0) || M(3,3) != T(1))
  {
    // not a similarity if bottom row is not [0,0,0,1]
    // TODO throw an exception here
    std::cerr << "third row of similarity matrix must be [0,0,0,1]"<<std::endl;
    return;
  }
  Eigen::Matrix<T,3,3> sr = M.template block<3,3>(0,0);
  this->scale_ = sr.determinant();
  if (this->scale_ <= T(0))
  {
    // similarity must have positive scale
    // TODO throw an exception
    std::cerr << "determinant in upper 3x3 of similarity matrix must be positive"<<std::endl;
    return;
  }
  // take the cube root
  this->scale_ = std::pow(this->scale_, static_cast<T>(1/3.0));
  // factor scale out of sr
  sr /= this->scale_;
  this->rot_ = rotation_<T>(sr);
  assert((Eigen::Matrix<T,3,3>(this->rot_)-sr).norm() < 1e-4);
  this->trans_ = M.template block<3,1>(3,0);
}


/// Convert to a 4x4 matrix
template <typename T>
similarity_<T>
::operator Eigen::Matrix<T,4,4>() const
{
  Eigen::Matrix<T,4,4> mat = Eigen::Matrix<T,4,4>::Zero();
  mat(3,3) = 1;
  mat.template block<3,3>(0,0) = this->scale_ * Eigen::Matrix<T,3,3>(this->rot_);
  mat.template block<3,1>(3,0) = this->trans_;
  return mat;
}


/// Compose two transforms
template <typename T>
similarity_<T>
similarity_<T>
::operator*(const similarity_<T>& rhs) const
{
  return similarity_<T>(this->scale_ * rhs.scale_,
                        this->rot_ * rhs.rot_,
                        *this * rhs.trans_);
}


/// Transform a vector
template <typename T>
Eigen::Matrix<T,3,1>
similarity_<T>
::operator*(const Eigen::Matrix<T,3,1>& rhs) const
{
  return this->scale_ * (this->rot_ * rhs) + this->trans_;
}


/// output stream operator for a similarity transformation
template <typename T>
std::ostream&  operator<<(std::ostream& s, const similarity_<T>& t)
{
  s << t.scale() << ' ' << t.rotation() << ' ' << t.translation();
  return s;
}


/// input stream operator for a similarity transformation
template <typename T>
std::istream&  operator>>(std::istream& s, similarity_<T>& t)
{
  T sc;
  rotation_<T> ro;
  vector_3_<T> tr;
  s >> sc >> ro >> tr;
  t = similarity_<T>(sc, ro, tr);
  return s;
}


/// \cond DoxygenSuppress
#define INSTANTIATE_SIMILARITY(T) \
template class MAPTK_LIB_EXPORT similarity_<T>; \
template MAPTK_LIB_EXPORT std::ostream&  operator<<(std::ostream& s, const similarity_<T>& t); \
template MAPTK_LIB_EXPORT std::istream&  operator>>(std::istream& s, similarity_<T>& t)

INSTANTIATE_SIMILARITY(double);
INSTANTIATE_SIMILARITY(float);

#undef INSTANTIATE_SIMILARITY
/// \endcond


} // end namespace maptk
