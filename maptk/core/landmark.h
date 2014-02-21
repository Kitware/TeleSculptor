/*ckwg +5
 * Copyright 2013-2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#ifndef MAPTK_LANDMARK_H_
#define MAPTK_LANDMARK_H_

#include "core_config.h"

#include <iostream>

#include <boost/shared_ptr.hpp>

#include "covariance.h"
#include "vector.h"


/**
 * \file
 * \brief Header for \link maptk::landmark landmark \endlink objects
 */


namespace maptk
{

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
  virtual ~landmark() {}

  /// Access the type info of the underlying data (double or float)
  virtual const std::type_info& data_type() const = 0;

  /// Accessor for the world coordinates
  virtual vector_3d loc() const = 0;
  /// Accessor for the landmark scale
  virtual double scale() const = 0;
  /// Accessor for the covariance
  virtual covariance_3d covar() const = 0;
};

/// Shared pointer type for the base landmark class
typedef boost::shared_ptr<landmark> landmark_sptr;

/// output stream operator for a base class landmark
/**
 * \param s output stream
 * \param m landmark to stream
 */
MAPTK_CORE_EXPORT std::ostream& operator<<(std::ostream& s, const landmark& m);


/// A representation of a 3D world point
template <typename T>
class MAPTK_CORE_EXPORT landmark_
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
  landmark_<T>(const vector_3_<T>& loc, T scale=1);

  /// Access staticly available type of underlying data (double or float)
  static const std::type_info& static_data_type() { return typeid(T); }
  /// Access the type info of the underlying data (double or float)
  virtual const std::type_info& data_type() const { return typeid(T); }

  /// Accessor for the world coordinates using underlying data type
  const vector_3_<T>& get_loc() const { return loc_; }
  /// Accessor for the world coordinates
  virtual vector_3d loc() const { return static_cast<vector_3d>(loc_); }

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
  void set_loc(const vector_3_<T>& loc) { loc_ = loc; }
  /// Set the scale of the landmark
  void set_scale(T scale) { scale_ = scale; }
  /// Set the covariance matrix of the landmark location
  void set_covar(const covariance_<3,T>& covar) { covar_ = covar; }

protected:
  /// landmark 3d position
  vector_3_<T> loc_;
  /// landmark scale
  T scale_;
  /// landmark covariance matrix
  covariance_<3,T> covar_;
};

/// Double precision landmark type
typedef landmark_<double> landmark_d;
/// Sincle precision landmark type
typedef landmark_<float> landmark_f;

/// output stream operator for a landmark
template <typename T>
MAPTK_CORE_EXPORT std::ostream& operator<<(std::ostream& s, const landmark_<T>& m);

/// input stream operator for a landmark
template <typename T>
MAPTK_CORE_EXPORT std::istream& operator>>(std::istream& s, landmark_<T>& m);


} // end namespace maptk


#endif // MAPTK_LANDMARK_H_
