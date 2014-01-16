/*ckwg +5
 * Copyright 2013-2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#ifndef MAPTK_CAMERA_H_
#define MAPTK_CAMERA_H_

#include "core_config.h"

#include <iostream>

#include "camera_intrinsics.h"
#include "covariance.h"
#include "rotation.h"
#include "vector.h"
#include <boost/shared_ptr.hpp>


namespace maptk
{


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
};

/// typedef for a camera shared pointer
typedef boost::shared_ptr<camera> camera_sptr;


/// A representation of a camera
/**
 * Contains camera location, orientation, and intrinsics
 */
template <typename T>
class MAPTK_CORE_EXPORT camera_ : public camera
{
public:
  /// Default Constructor
  camera_<T>()
  : center_(T(0), T(0), T(0)),
    orientation_(),
    intrinsics_()
  {}

  /// Constructor - from camera center, rotation, and intrinsics
  camera_<T>(const vector_3_<T>& center,
             const rotation_<T>& rotation,
             const camera_intrinsics_<T>& intrincs = camera_intrinsics_<T>())
  : center_(center),
    orientation_(rotation),
    intrinsics_(intrincs)
  {}

  /// Copy Constructor from another type
  template <typename U>
  explicit camera_<T>(const camera_<U>& other)
  : center_(static_cast<vector_3_<T> >(other.get_center())),
    center_covar_(static_cast<covariance_<3,T> >(other.get_center_covar())),
    orientation_(static_cast<rotation_<T> >(get_rotation())),
    intrinsics_(static_cast<camera_intrinsics_<T> >(get_intrinsics()))
  {}

  /// Access staticly available type of underlying data (double or float)
  static const std::type_info& static_data_type() { return typeid(T); }
  /// Access the type info of the underlying data (double or float)
  virtual const std::type_info& data_type() const { return typeid(T); }

  /// Accessor for the camera center of projection (position)
  virtual vector_3d center() const
  { return static_cast<vector_3d>(center_); }
  /// Accessor for the translation vector
  virtual vector_3d translation() const
  { return static_cast<vector_3d>(get_translation()); }
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
  const vector_3_<T>& get_center() const { return center_; }
  /// Accessor for the translation vector using underlying data type
  vector_3_<T> get_translation() const { return - (orientation_ * center_); }
  /// Accessor for the covariance of camera center using underlying data type
  const covariance_<3,T>& get_center_covar() const { return center_covar_; }
  /// Accessor for the rotation using underlying data type
  const rotation_<T>& get_rotation() const { return orientation_; }
  /// Accessor for the intrinsics using underlying data type
  const camera_intrinsics_<T>& get_intrinsics() const { return intrinsics_; }

  /// Set the camera center of projection
  void set_center(const vector_3_<T>& center) { center_ = center; }
  /// Set the translation vector (relative to current rotation)
  void set_translation(const vector_3_<T>& translation)
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
   * the vertical image direction is closest to \ref up_direction in the world.
   * \param [in] stare_point the location at which the camera is oriented to point
   * \param [in] up_direction the vector which is "up" in the world (defaults to Z-axis)
   */
  void look_at(const vector_3_<T>& stare_point,
               const vector_3_<T>& up_direction=vector_3_<T>(0,0,1) );
protected:
  /// The camera center of project
  vector_3_<T> center_;
  /// The covariance of the camera center location
  covariance_<3,T> center_covar_;
  /// The camera rotation
  rotation_<T> orientation_;
  /// The camera intrinics
  camera_intrinsics_<T> intrinsics_;
};


typedef camera_<double> camera_d;
typedef camera_<float> camera_f;


/// output stream operator for a camera
template <typename T>
MAPTK_CORE_EXPORT std::ostream& operator<<(std::ostream& s, const camera_<T>& c);

/// input stream operator for a camera
template <typename T>
MAPTK_CORE_EXPORT std::istream& operator>>(std::istream& s, camera_<T>& c);


} // end namespace maptk


#endif // MAPTK_CAMERA_H_
