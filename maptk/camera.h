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
 * \brief Header for \link maptk::camera camera \endlink and
 *        \link maptk::camera_ camera_<T> \endlink classes
 */

#ifndef MAPTK_CAMERA_H_
#define MAPTK_CAMERA_H_

#include <maptk/config.h>

#include <iostream>
#include <vector>

#include "camera_intrinsics.h"
#include "covariance.h"
#include "rotation.h"
#include "vector.h"
#include "similarity.h"
#include <boost/shared_ptr.hpp>


namespace maptk
{

/// forward declaration of camera class
class camera;
/// typedef for a camera shared pointer
typedef boost::shared_ptr<camera> camera_sptr;

/// An abstract representation of camera
/**
 * The base class of cameras is abstract and provides a
 * double precision interface.  The templated derived class
 * can store values in either single or double precision.
 */
class camera
{
public:
  /// Destructor
  virtual ~camera() {}

  /// Create a clone of this camera object
  virtual camera_sptr clone() const = 0;

  /// Access the type info of the underlying data (double or float)
  virtual const std::type_info& data_type() const = 0;

  /// Accessor for the camera center of projection (position)
  virtual vector_3d center() const = 0;
  /// Accessor for the translation vector
  virtual vector_3d translation() const = 0;
  /// Accessor for the covariance of camera center
  virtual covariance_3d center_covar() const = 0;
  /// Accessor for the rotation
  virtual rotation_d rotation() const = 0;
  /// Accessor for the intrinsics
  virtual camera_intrinsics_d intrinsics() const = 0;

  /// Apply a similarity transformation to the camera in place
  virtual void transform(const similarity_d& xform) = 0;
};

/// output stream operator for a base class camera
MAPTK_LIB_EXPORT std::ostream& operator<<(std::ostream& s, const camera& c);


/// A representation of a camera
/**
 * Contains camera location, orientation, and intrinsics
 */
template <typename T>
class MAPTK_LIB_EXPORT camera_
  : public camera
{
public:
  /// Default Constructor
  camera_<T>()
  : center_(T(0), T(0), T(0)),
    orientation_(),
    intrinsics_()
  {}

  /// Constructor - from camera center, rotation, and intrinsics
  camera_<T>(const Eigen::Matrix<T,3,1>& center,
             const rotation_<T>& rotation,
             const camera_intrinsics_<T>& intrinsics = camera_intrinsics_<T>())
  : center_(center),
    orientation_(rotation),
    intrinsics_(intrinsics)
  {}

  /// Copy Constructor from another type
  template <typename U>
  explicit camera_<T>(const camera_<U>& other)
  : center_(other.get_center().template cast<T>()),
    center_covar_(static_cast<covariance_<3,T> >(other.get_center_covar())),
    orientation_(static_cast<rotation_<T> >(get_rotation())),
    intrinsics_(static_cast<camera_intrinsics_<T> >(get_intrinsics()))
  {}

  /// Create a clone of this camera object
  virtual camera_sptr clone() const
  { return camera_sptr(new camera_<T>(*this)); }

  /// Access staticly available type of underlying data (double or float)
  static const std::type_info& static_data_type() { return typeid(T); }
  /// Access the type info of the underlying data (double or float)
  virtual const std::type_info& data_type() const { return typeid(T); }

  /// Accessor for the camera center of projection (position)
  virtual vector_3d center() const
  { return center_.template cast<double>(); }
  /// Accessor for the translation vector
  virtual vector_3d translation() const
  { return get_translation().template cast<double>(); }
  /// Accessor for the covariance of camera center
  virtual covariance_3d center_covar() const
  { return static_cast<covariance_3d>(center_covar_); }
  /// Accessor for the rotation
  virtual rotation_d rotation() const
  { return static_cast<rotation_d>(orientation_); }
  /// Accessor for the intrinsics
  virtual camera_intrinsics_d intrinsics() const
  { return static_cast<camera_intrinsics_d>(intrinsics_); }

  /// Accessor for the camera center of projection using underlying data type
  const Eigen::Matrix<T,3,1> & get_center() const { return center_; }
  /// Accessor for the translation vector using underlying data type
  Eigen::Matrix<T,3,1> get_translation() const { return - (orientation_ * center_); }
  /// Accessor for the covariance of camera center using underlying data type
  const covariance_<3,T>& get_center_covar() const { return center_covar_; }
  /// Accessor for the rotation using underlying data type
  const rotation_<T>& get_rotation() const { return orientation_; }
  /// Accessor for the intrinsics using underlying data type
  const camera_intrinsics_<T>& get_intrinsics() const { return intrinsics_; }

  /// Set the camera center of projection
  void set_center(const Eigen::Matrix<T,3,1>& center) { center_ = center; }
  /// Set the translation vector (relative to current rotation)
  void set_translation(const Eigen::Matrix<T,3,1>& translation)
  {
    center_ = - (orientation_.inverse() * translation);
  }
  /// Set the covariance matrix of the feature
  void set_center_covar(const covariance_<3,T>& center_covar) { center_covar_ = center_covar; }
  /// Set the rotation
  void set_rotation(const rotation_<T>& rotation) { orientation_ = rotation; }
  /// Set the intrinsics
  void set_intrinsics(const camera_intrinsics_<T>& intrinsics) { intrinsics_ = intrinsics; }

  /// Rotate the camera about its center such that it looks at the given point.
  /**
   * The camera should also be rotated about its principal axis such that
   * the vertical image direction is closest to \c up_direction in the world.
   * \param [in] stare_point the location at which the camera is oriented to point
   * \param [in] up_direction the vector which is "up" in the world (defaults to Z-axis)
   */
  void look_at(const Eigen::Matrix<T, 3, 1>& stare_point,
               const Eigen::Matrix<T, 3, 1>& up_direction=Eigen::Vector3d::UnitZ() );

  /// Convert to a 3x4 homogeneous projection matrix
  operator Eigen::Matrix<T,3,4>() const;

  /// Project a 3D point into a 2D image point
  Eigen::Matrix<T, 2, 1> project(const Eigen::Matrix<T, 3, 1>& pt) const;

  /// Apply a similarity transformation to the camera in place
  virtual void transform(const similarity_d& xform)
  { apply_transform(similarity_<T>(xform)); }

  /// Transform the camera by applying a similarity transformation in place
  camera_<T>& apply_transform(const similarity_<T>& xform);

protected:
  /// The camera center of project
  Eigen::Matrix<T,3,1> center_;
  /// The covariance of the camera center location
  covariance_<3,T> center_covar_;
  /// The camera rotation
  rotation_<T> orientation_;
  /// The camera intrinics
  camera_intrinsics_<T> intrinsics_;
};


/// Double-precision camera type
typedef camera_<double> camera_d;
/// Single-precision camera type
typedef camera_<float> camera_f;


/// output stream operator for a camera
/**
 * \param s output stream
 * \param c camera to stream
 */
template <typename T>
MAPTK_LIB_EXPORT std::ostream& operator<<(std::ostream& s, const camera_<T>& c);

/// input stream operator for a camera
/**
 * \param s input stream
 * \param c camera to stream into
 */
template <typename T>
MAPTK_LIB_EXPORT std::istream& operator>>(std::istream& s, camera_<T>& c);


/// Generate an interpolated camera between \c A and \c B by a given fraction \c f
/**
 * \c f should be 0 < \c f < 1. A value outside this range is valid, but \c f
 * must not be 0.
 *
 * \param A Camera to interpolate from.
 * \param B Camera to interpolate to.
 * \param f Decimal fraction in between A and B for the returned camera to represent.
 */
template <typename T>
MAPTK_LIB_EXPORT
camera_<T> interpolate_camera(camera_<T> const& A, camera_<T> const& B, T f);


/// Genreate an interpolated camera from sptrs
/**
 * \relatesalso interpolate_camera
 *
 * TODO: Deprecate this function by replacing its use with a tiered dynamic
 * cast to either camera_<double> or camera_<float>, using the one that
 * doesn't produce a NULL pointer.
 */
MAPTK_LIB_EXPORT
camera_sptr interpolate_camera(camera_sptr A, camera_sptr B, double f);


/// Generate N evenly interpolated cameras in between \c A and \c B
/**
 * \c n must be >= 1.
 */
template <typename T>
MAPTK_LIB_EXPORT
void interpolated_cameras(camera_<T> const& A,
                          camera_<T> const& B,
                          size_t n,
                          std::vector< camera_<T> > & interp_cams);


} // end namespace maptk


#endif // MAPTK_CAMERA_H_
