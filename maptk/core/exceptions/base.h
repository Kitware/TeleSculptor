/*ckwg +5
 * Copyright 2013-2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

/**
 * \file
 * \brief MAPTK base exception interface
 */

#ifndef MAPTK_CORE_EXCEPTIONS_BASE_H
#define MAPTK_CORE_EXCEPTIONS_BASE_H

#include <maptk/core/core_config.h>
#include <string>

namespace maptk
{

/// The base class for all maptk/core exceptions
/**
 * \ingroup exceptions
 */
class MAPTK_CORE_EXPORT maptk_core_base_exception
  : public std::exception
{
  public:
    /// Constructor
    maptk_core_base_exception() MAPTK_NOTHROW;
    /// Destructor
    virtual ~maptk_core_base_exception() MAPTK_NOTHROW;

    /// Description of the exception
    /**
     * \returns A string describing what went wrong.
     */
    char const* what() const MAPTK_NOTHROW;
  protected:
    /// descriptive string as to what happened to cause the exception.
    std::string m_what;
};

/// Exception for incorrect input values
/**
 * \ingroup exceptions
 */
class MAPTK_CORE_EXPORT invalid_value
  : public maptk_core_base_exception
{
public:
  /// Constructor
  invalid_value(std::string reason) MAPTK_NOTHROW;
  /// Destructor
  virtual ~invalid_value() MAPTK_NOTHROW;
protected:
  /// Reason for invalidity
  std::string m_reason;
};

} // end namespace maptk

#endif // MAPTK_CORE_EXCEPTIONS_BASE_H
