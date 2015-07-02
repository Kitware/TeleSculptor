/*ckwg +29
 * Copyright 2013-2015 by Kitware, Inc.
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
 * \brief Header for \link maptk::landmark landmark \endlink objects
 */

#ifndef MAPTK_LANDMARK_H_
#define MAPTK_LANDMARK_H_

#include <maptk/config.h>

#include <iostream>

#include <boost/shared_ptr.hpp>

#include "covariance.h"
#include "vector.h"
#include "similarity.h"


namespace maptk
{


/// forward declaration of landmark class
class landmark;
/// typedef for a landmark shared pointer
typedef boost::shared_ptr<landmark> landmark_sptr;

/// An abstract representation of a 3D world point.
/**
 * The base class landmark is abstract and provides a
 * double precision interface.  The templated derived class
 * can store values in either single or double precision.
 */
class landmark
{
public:
  /// Destructor
  virtual ~landmark() MAPTK_DEFAULT_DTOR;

  /// Create a clone of this landmark object
  virtual landmark_sptr clone() const = 0;

  /// Access the type info of the underlying data (double or float)
  virtual const std::type_info& data_type() const = 0;

  /// Accessor for the world coordinates
  virtual vector_3d loc() const = 0;
  /// Accessor for the landmark scale
  virtual double scale() const = 0;
  /// Accessor for the covariance
  virtual covariance_3d covar() const = 0;

  /// Apply a similarity transformation to the landmark in place
  virtual void transform(const similarity_d& xform) = 0;
};

/// output stream operator for a base class landmark
/**
 * \param s output stream
 * \param m landmark to stream
 */
MAPTK_LIB_EXPORT std::ostream& operator<<(std::ostream& s, const landmark& m);


/// A representation of a 3D world point
template <typename T>
class MAPTK_LIB_EXPORT landmark_
  : public landmark
{
public:
  /// Default Constructor
  landmark_<T>();

  /// Constructor for a landmark
  /**
   * \param loc 3D location of the landmark
   * \param scale optional scale of the landmark (default of 1)
   */
  landmark_<T>(const Eigen::Matrix<T,3,1>& loc, T scale=1);

  /// Create a clone of this landmark object
  virtual landmark_sptr clone() const
  { return landmark_sptr(new landmark_<T>(*this)); }

  /// Access staticly available type of underlying data (double or float)
  static const std::type_info& static_data_type() { return typeid(T); }
  /// Access the type info of the underlying data (double or float)
  virtual const std::type_info& data_type() const { return typeid(T); }

  /// Accessor for the world coordinates using underlying data type
  const Eigen::Matrix<T,3,1>& get_loc() const { return loc_; }
  /// Accessor for the world coordinates
  virtual vector_3d loc() const { return loc_.template cast<double>(); }

  /// Accessor for the landmark scale using underlying data type
  T get_scale() const { return scale_; }
  /// Accessor for the landmark scale
  virtual double scale() const { return static_cast<double>(scale_); }

  /// Accessor for the covariance using underlying data type
  const covariance_<3,T>& get_covar() const { return covar_; }
  /// Accessor for the covariance
  virtual covariance_3d covar() const { return static_cast<covariance_3d>(covar_); }

  // Set the landmark position in image space
  /**
   * \param loc new location of this landmark
   */
  void set_loc(const Eigen::Matrix<T,3,1>& loc) { loc_ = loc; }
  /// Set the scale of the landmark
  void set_scale(T scale) { scale_ = scale; }
  /// Set the covariance matrix of the landmark location
  void set_covar(const covariance_<3,T>& covar) { covar_ = covar; }

  /// Apply a similarity transformation to the landmark in place
  virtual void transform(const similarity_d& xform)
  { apply_transform(similarity_<T>(xform)); }

  /// Transform the landmark by applying a similarity transformation in place
  landmark_<T>& apply_transform(const similarity_<T>& xform);

protected:

  /// A vector representing the 3D position of the landmark
  Eigen::Matrix<T,3,1> loc_;
  /// The scale of the landmark in 3D
  T scale_;
  /// Covariance representing uncertainty in the estimate of 3D position
  covariance_<3,T> covar_;
};


/// A double precision landmark
typedef landmark_<double> landmark_d;
/// A single precision landmark
typedef landmark_<float> landmark_f;

/// output stream operator for a landmark
template <typename T>
MAPTK_LIB_EXPORT std::ostream& operator<<(std::ostream& s, const landmark_<T>& m);

/// input stream operator for a landmark
template <typename T>
MAPTK_LIB_EXPORT std::istream& operator>>(std::istream& s, landmark_<T>& m);


} // end namespace maptk


#endif // MAPTK_LANDMARK_H_
