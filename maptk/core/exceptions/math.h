/*ckwg +5
 * Copyright 2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

/**
 * \file
 * \brief MAPTK Exceptions pertaining to math operations
 */

#ifndef MAPTK_CORE_EXCEPTIONS_MATH_H
#define MAPTK_CORE_EXCEPTIONS_MATH_H

#include "base.h"
#include <string>
#include <maptk/core/types.h>

namespace maptk
{


/// MAPTK Generic math exception
class MAPTK_CORE_EXPORT math_exception
  : public maptk_core_base_exception
{
public:
  /// Constructor
  math_exception() MAPTK_NOTHROW;
  /// Destructor
  virtual ~math_exception() MAPTK_NOTHROW;
};


/// Exception for when a matrix is non-invertible
class MAPTK_CORE_EXPORT non_invertible_matrix
  : public math_exception
{
public:
  /// Constructor
  non_invertible_matrix() MAPTK_NOTHROW;
  /// Destructor
  virtual ~non_invertible_matrix() MAPTK_NOTHROW;
};


/// Exception for when some point maps to infinity
class MAPTK_CORE_EXPORT point_maps_to_infinity
  : public math_exception
{
public:
  /// Constructor
  point_maps_to_infinity() MAPTK_NOTHROW;
  /// Destructor
  virtual ~point_maps_to_infinity() MAPTK_NOTHROW;
};


/// We cannot perfom some operation on a matrix
class MAPTK_CORE_EXPORT invalid_matrix_operation
  : public math_exception
{
public:
  /// Constructor
  /*
   * \param reason  The reason for invalidity.
   */
  invalid_matrix_operation(std::string reason) MAPTK_NOTHROW;
  /// Destructor
  virtual ~invalid_matrix_operation() MAPTK_NOTHROW;

  /// Reason the operation is invalid
  std::string m_reason;
};


} // end maptk namespace

#endif // MAPTK_CORE_EXCEPTIONS_MATH_H
