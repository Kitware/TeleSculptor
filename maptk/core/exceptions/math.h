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


} // end maptk namespace

#endif // MAPTK_CORE_EXCEPTIONS_MATH_H
