/*ckwg +5
 * Copyright 2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
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
::similarity_(const matrix_<4,4,T>& M)
{
  if (M(3,0) != T(0) || M(3,1) != T(0) || M(3,2) != T(0) || M(3,3) != T(1))
  {
    // not a similarity if bottom row is not [0,0,0,1]
    // TODO throw an exception here
    std::cerr << "third row of similarity matrix must be [0,0,0,1]"<<std::endl;
    return;
  }
  matrix_<3,3,T> sr;
  M.extract(sr);
  this->scale_ = determinant(sr);
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
  assert((matrix_<3,3,T>(this->rot_)-sr).frobenius_norm() < 1e-4);
  M.column(3).extract(this->trans_);
}


/// Convert to a 4x4 matrix
template <typename T>
similarity_<T>
::operator matrix_<4,4,T>() const
{
  matrix_<4,4,T> mat(T(0));
  mat(3,3) = 1;
  mat.update(this->scale_ * matrix_<3,3,T>(this->rot_));
  mat.set_column(3, mat.column(3).update(this->trans_));
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
vector_<3,T>
similarity_<T>
::operator*(const vector_<3,T>& rhs) const
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
template class MAPTK_CORE_EXPORT similarity_<T>; \
template MAPTK_CORE_EXPORT std::ostream&  operator<<(std::ostream& s, const similarity_<T>& t); \
template MAPTK_CORE_EXPORT std::istream&  operator>>(std::istream& s, similarity_<T>& t)

INSTANTIATE_SIMILARITY(double);
INSTANTIATE_SIMILARITY(float);

#undef INSTANTIATE_SIMILARITY
/// \endcond


} // end namespace maptk
