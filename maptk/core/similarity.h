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
 * \brief Header for \link maptk::similarity_ similarity_<T> \endlink class
 *        for similarity transformations
 */

#ifndef MAPTK_SIMILARITY_H_
#define MAPTK_SIMILARITY_H_

#include <iostream>

#include "matrix.h"
#include "vector.h"
#include "rotation.h"


namespace maptk
{

/// A representation of a 3D similarity transformation.
/**
 * A similarity transformation is one that includes a scaling, rotation,
 * and translation
 */
template <typename T>
class MAPTK_CORE_EXPORT similarity_
{
public:
  /// Default Constructor
  similarity_<T>()
  : scale_(1),
    rot_(),
    trans_(0,0,0) {}

  /// Copy Constructor from another type
  template <typename U>
  explicit similarity_<T>(const similarity_<U>& other)
  : scale_(static_cast<T>(other.scale())),
    rot_(static_cast<rotation_<T> >(other.rotation())),
    trans_(static_cast<vector_3_<T> >(other.translation())) {}

  /// Constructor - from scale, rotatation, and translation
  /**
   * \param s the scale factor
   * \param r the rotation
   * \param t the translation vector
   */
  similarity_<T>(const T& s, const rotation_<T>& r,
                 const vector_<3,T>& t)
  : scale_(s),
    rot_(r),
    trans_(t) {}

  /// Constructor - from a matrix
  /**
   * requires a matrix which represents a similarity tranformation
   * in homogeneous coordinates
   * \param mat Transform in matrix form to initialize from.
   */
  explicit similarity_<T>(const matrix_<4,4,T>& mat);

  /// Convert to a 4x4 matrix
  operator matrix_<4,4,T>() const;

  /// Return scale factor
  const T& scale() const { return scale_; }

  /// Return the rotation
  const rotation_<T>& rotation() const { return rot_; }

  /// Return the translation vector
  const vector_3_<T>& translation() const { return trans_; }

  /// Compute the inverse similarity
  similarity_<T> inverse() const
  {
    T inv_scale = T(1) / scale_;
    rotation_<T> inv_rot(rot_.inverse());
    return similarity_<T>(inv_scale, inv_rot, -inv_scale * (inv_rot * trans_));
  }

  /// Compose two similarities
  /**
   * \param rhs other similarity to compose with.
   */
  similarity_<T> operator*(const similarity_<T>& rhs) const;

  /// Transform a vector
  /**
   * \note for a large number of vectors, it is more efficient to
   *       create a transform matrix and use matrix multiplcation
   * \param rhs vector to transform.
   */
  vector_<3,T> operator*(const vector_<3,T>& rhs) const;

  /// Equality operator
  inline bool operator==(const similarity_<T>& rhs) const
  {
    return this->scale_ == rhs.scale_ &&
           this->rot_   == rhs.rot_   &&
           this->trans_ == rhs.trans_;
  }

  /// Inequality operator
  inline bool operator!=(const similarity_<T>& rhs) const
  {
    return !(*this == rhs);
  }

protected:
  /// scale factor
  T scale_;
  /// rotation
  rotation_<T> rot_;
  /// translation
  vector_3_<T> trans_;
};


/// \cond DoxygenSuppress
typedef similarity_<double> similarity_d;
typedef similarity_<float> similarity_f;
/// \endcond


/// output stream operator for a similarity transformation
template <typename T>
MAPTK_CORE_EXPORT std::ostream&  operator<<(std::ostream& s, const similarity_<T>& t);

/// input stream operator for a similarity transformation
template <typename T>
MAPTK_CORE_EXPORT std::istream&  operator>>(std::istream& s, similarity_<T>& t);


} // end namespace maptk


#endif // MAPTK_SIMILARITY_H_
