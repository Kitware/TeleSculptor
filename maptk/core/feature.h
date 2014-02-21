/*ckwg +5
 * Copyright 2013-2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

/**
 * \file
 * \brief core feature iterface
 */

#ifndef MAPTK_FEATURE_H_
#define MAPTK_FEATURE_H_

#include "core_config.h"

#include <iostream>
#include <typeinfo>

#include <boost/shared_ptr.hpp>

#include "covariance.h"
#include "vector.h"

namespace maptk
{

/// A representation of a 2D image feature point.
///
/// The base class of features is abstract and provides a
/// double precision interface.  The templated derived class
/// can store values in either single or double precision.
class feature
{
public:
  /// Destructor
  virtual ~feature() {}

  /// Access the type info of the underlying data (double or float)
  virtual const std::type_info& data_type() const = 0;

  /// Accessor for the image coordinates
  virtual vector_2d loc() const = 0;
  /// Accessor for the feature magnitude
  virtual double magnitude() const = 0;
  /// Accessor for the feature scale
  virtual double scale() const = 0;
  /// Accessor for the feature angle
  virtual double angle() const = 0;
  /// Accessor for the covariance
  virtual covariance_2d covar() const = 0;
};

/// Shared pointer for base feature type
typedef boost::shared_ptr<feature> feature_sptr;

/// output stream operator for base class feature
/**
 * \param s output stream
 * \param f feature to stream
 */
MAPTK_CORE_EXPORT std::ostream& operator<<(std::ostream& s, const feature& f);


/// A concrete 2D image feature point.
///
/// Templated over real number type (double or float).
template <typename T>
class MAPTK_CORE_EXPORT feature_
  : public feature
{
public:
  /// Default Constructor
  feature_<T>();

  /// Constructor for a feature
  feature_<T>(const vector_2_<T>& loc, T mag=0.0,
              T scale=1.0, T angle=0.0);

  /// Access staticly available type of underlying data (double or float)
  static const std::type_info& static_data_type() { return typeid(T); }
  /// Access the type info of the underlying data (double or float)
  virtual const std::type_info& data_type() const { return typeid(T); }

  /// Accessor for the image coordinates using underlying data type
  const vector_2_<T>& get_loc() const { return loc_; }
  /// Accessor for the image coordinates
  virtual vector_2d loc() const { return static_cast<vector_2d>(loc_); }

  /// Accessor for the feature magnitude using underlying data type
  T get_magnitude() const { return magnitude_; }
  /// Accessor for the feature magnitude
  virtual double magnitude() const { return static_cast<double>(magnitude_); }

  /// Accessor for the feature scale using underlying data type
  T get_scale() const { return scale_; }
  /// Accessor for the feature scale
  virtual double scale() const { return static_cast<double>(scale_); }

  /// Accessor for the feature angle using underlying data type
  T get_angle() const { return angle_; }
  /// Accessor for the feature angle
  virtual double angle() const { return static_cast<double>(angle_); }

  /// Accessor for the covariance using underlying data type
  const covariance_<2,T>& get_covar() const { return covar_; }
  /// Accessor for the covariance
  virtual covariance_2d covar() const { return static_cast<covariance_2d>(covar_); }


  /// Set the feature position in image space
  void set_loc(const vector_2_<T>& loc) { loc_ = loc; }
  /// Set the magnitude of the feature response
  void set_magnitude(T magnitude) { magnitude_ = magnitude; }
  /// Set the scale of the feature
  void set_scale(T scale) { scale_ = scale; }
  /// Set the angle of the feature
  void set_angle(T angle) { angle_ = angle; }
  /// Set the covariance matrix of the feature
  void set_covar(const covariance_<2,T>& covar) { covar_ = covar; }

protected:

  /// location of feature
  vector_2_<T> loc_;
  /// magnitude of feature
  T magnitude_;
  /// scale of feature
  T scale_;
  /// angle of feature
  T angle_;
  /// covariance matrix of feature
  covariance_<2,T> covar_;
};

/// Double-precision feature_ type
typedef feature_<double> feature_d;
/// Single-precision feature_ type
typedef feature_<float> feature_f;

/// output stream operator for a feature
template <typename T>
MAPTK_CORE_EXPORT std::ostream& operator<<(std::ostream& s, const feature_<T>& f);

/// input stream operator for a feature
template <typename T>
MAPTK_CORE_EXPORT std::istream& operator>>(std::istream& s, feature_<T>& f);

} // end namespace maptk


#endif // MAPTK_FEATURE_H_
