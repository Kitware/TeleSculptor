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
 * \brief Header for \link maptk::vector_ vector_<N,T> \endlink class
 *        as well as \link maptk::vector_2_ vector_2_<T> \endlink,
 *        \link maptk::vector_3_ vector_3_<T> \endlink, and
 *        \link maptk::vector_4_ vector_4_<T> \endlink classes
 */

#ifndef MAPTK_VECTOR_H_
#define MAPTK_VECTOR_H_

#include <maptk/config.h>

#include <cstring>
#include <iostream>
#include <cassert>

#include <Eigen/Core>


namespace maptk
{


/// Compute the 2D cross product
/**
 * \relatesalso vector_
 * \param v1 a vector
 * \param v2 another vector
 */
template <typename T>
inline T
cross_product(const Eigen::Matrix<T,2,1>& v1,
              const Eigen::Matrix<T,2,1>& v2)
{
  return v1[0] * v2[1] - v1[1] * v2[0];
}


/// Compute the 3D cross product
/**
 * \relatesalso vector_
 * \param v1 a vector
 * \param v2 another vector
 */
template <typename T>
inline Eigen::Matrix<T,3,1>
cross_product(const Eigen::Matrix<T,3,1>& v1,
              const Eigen::Matrix<T,3,1>& v2)
{
  Eigen::Matrix<T,3,1> result;
  result[0] = v1[1] * v2[2] - v1[2] * v2[1];
  result[1] = v1[2] * v2[0] - v1[0] * v2[2];
  result[2] = v1[0] * v2[1] - v1[1] * v2[0];
  return result;
}


/// A representation of a 2D vector.
/**
 * This derived class exists to add convenience
 * constructors and accessors
 */
template <typename T>
class vector_2_ : public Eigen::Matrix<T,2,1>
{
public:
  /// Default Constructor
  vector_2_<T> () {}

  /// Copy Constructor
  vector_2_<T> (const vector_2_<T>& other)
  : Eigen::Matrix<T,2,1>(other) {}

  /// Constructor from base class
  template<typename OtherDerived>
  vector_2_<T> (const Eigen::MatrixBase<OtherDerived>& base)
  : Eigen::Matrix<T,2,1>(base) {}

  /// Constructor from another data type
  template <typename U>
  explicit vector_2_<T> (const vector_2_<U>& other)
  : Eigen::Matrix<T,2,1>(other.template cast<T>()) {}

  /// Constructor for a 2D vector
  vector_2_<T>(const T& x, const T& y)
  {
    (*this)[0] = x;
    (*this)[1] = y;
  }

  /// Assignement operator
  template <typename OtherDerived>
  vector_2_<T>& operator= (const Eigen::MatrixBase<OtherDerived>& other)
  {
    this->Eigen::Matrix<T,2,1>::operator=(other);
    return *this;
  }

  /// Accessor for the X coordinate
  T& x() { return (*this)[0]; }
  /// Accessor for the X coordinate (const)
  const T& x() const { return (*this)[0]; }
  /// Accessor for the Y coordinate
  T& y() { return (*this)[1]; }
  /// Accessor for the Y coordinate (const)
  const T& y() const { return (*this)[1]; }
};


/// A representation of a 3D vector.
/**
 * This derived class exists to add convenience
 * constructors and accessors
 */
template <typename T>
class vector_3_ : public Eigen::Matrix<T,3,1>
{
public:
  /// Default Constructor
  vector_3_<T> () {}

  /// Copy Constructor
  vector_3_<T> (const vector_3_<T>& other)
  : Eigen::Matrix<T,3,1>(other) {}

  /// Constructor from base class
  template <typename OtherDerived>
  vector_3_<T> (const Eigen::MatrixBase<OtherDerived>& base)
  : Eigen::Matrix<T,3,1>(base) {}

  /// Constructor from another data type
  template <typename U>
  explicit vector_3_<T> (const vector_3_<U>& other)
  : Eigen::Matrix<T,3,1>(other.template cast<T>()) {}

  /// Constructor for a 3D vector
  vector_3_<T>(const T& x, const T& y, const T& z)
  {
    (*this)[0] = x;
    (*this)[1] = y;
    (*this)[2] = z;
  }

  /// Assignement operator
  template <typename OtherDerived>
  vector_3_<T>& operator= (const Eigen::MatrixBase<OtherDerived>& other)
  {
    this->Eigen::Matrix<T,3,1>::operator=(other);
    return *this;
  }

  /// Accessor for the X coordinate
  T& x() { return (*this)[0]; }
  /// Accessor for the X coordinate (const)
  const T& x() const { return (*this)[0]; }
  /// Accessor for the Y coordinate
  T& y() { return (*this)[1]; }
  /// Accessor for the Y coordinate (const)
  const T& y() const { return (*this)[1]; }
  /// Accessor for the Z coordinate
  T& z() { return (*this)[2]; }
  /// Accessor for the Z coordinate (const)
  const T& z() const { return (*this)[2]; }
};


/// A representation of a 4D vector.
/**
 * This derived class exists to add convenience
 * constructors and accessors
 */
template <typename T>
class vector_4_ : public Eigen::Matrix<T,4,1>
{
public:
  /// Default Constructor
  vector_4_<T> () {}

  /// Copy Constructor
  vector_4_<T> (const vector_4_<T>& other)
  : Eigen::Matrix<T,4,1>(other) {}

  /// Constructor from base class
  template <typename OtherDerived>
  vector_4_<T> (const Eigen::MatrixBase<OtherDerived>& base)
  : Eigen::Matrix<T,4,1>(base) {}

  /// Constructor from another data type
  template <typename U>
  explicit vector_4_<T> (const vector_4_<U>& other)
  : Eigen::Matrix<T,4,1>(other.template cast<T>()) {}

  /// Constructor for a 4D vector
  vector_4_<T>(const T& x, const T& y, const T& z, const T& w)
  {
    (*this)[0] = x;
    (*this)[1] = y;
    (*this)[2] = z;
    (*this)[3] = w;
  }

  /// Assignement operator
  template <typename OtherDerived>
  vector_4_<T>& operator= (const Eigen::MatrixBase<OtherDerived>& other)
  {
    this->Eigen::Matrix<T,4,1>::operator=(other);
    return *this;
  }

  /// Accessor for the X coordinate
  T& x() { return (*this)[0]; }
  /// Accessor for the X coordinate (const)
  const T& x() const { return (*this)[0]; }
  /// Accessor for the Y coordinate
  T& y() { return (*this)[1]; }
  /// Accessor for the Y coordinate (const)
  const T& y() const { return (*this)[1]; }
  /// Accessor for the Z coordinate
  T& z() { return (*this)[2]; }
  /// Accessor for the Z coordinate (const)
  const T& z() const { return (*this)[2]; }
  /// Accessor for the W coordinate
  T& w() { return (*this)[3]; }
  /// Accessor for the W coordinate (const)
  const T& w() const { return (*this)[3]; }
};


/// \cond DoxygenSuppress
typedef vector_2_<double> vector_2d;
typedef vector_2_<float>  vector_2f;
typedef vector_3_<double> vector_3d;
typedef vector_3_<float>  vector_3f;
typedef vector_4_<double> vector_4d;
typedef vector_4_<float>  vector_4f;
/// \endcond


/// output stream operator for a vector
/**
 * \param s an output stream
 * \param v vector to stream
 */
template <typename T, int N>
MAPTK_LIB_EXPORT std::ostream&  operator<<(std::ostream& s, const Eigen::Matrix<T,N,1>& v);

/// input stream operator for a vector
/**
 * \param s an input stream
 * \param v vector to stream into
 */
template <typename T, int N>
MAPTK_LIB_EXPORT std::istream&  operator>>(std::istream& s, Eigen::Matrix<T,N,1>& v);


} // end namespace maptk


#endif // MAPTK_VECTOR_H_
