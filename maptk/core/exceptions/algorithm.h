/*ckwg +5
 * Copyright 2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#ifndef MAPTK_CORE_EXCEPTIONS_ALGORITHM_H
#define MAPTK_CORE_EXCEPTIONS_ALGORITHM_H

#include "base.h"
#include <string>

namespace maptk
{

/// Base class for all algorithm related exceptions
class MAPTK_CORE_EXPORT algorithm_exception
  : public maptk_core_base_exception
{
  public:
    /// Constructor
    algorithm_exception(std::string type,
                        std::string impl,
                        std::string reason) MAPTK_NOTHROW;
    /// Deconstructor
    virtual ~algorithm_exception() MAPTK_NOTHROW;

    /// The name of the algorithm type
    std::string m_algo_type;
    /// The name of the algorithm implementation
    std::string m_algo_impl;
    /// String explanation of the reason for the exception
    std::string m_reason;
};

/// Exception for when an algorithm receives an invalid configuration
class MAPTK_CORE_EXPORT algorithm_configuration_exception
  : public algorithm_exception
{
  public:
    algorithm_configuration_exception(std::string type,
                                      std::string impl,
                                      std::string reason) MAPTK_NOTHROW;
    virtual ~algorithm_configuration_exception() MAPTK_NOTHROW;
};

/// Exception for when checking an invalid impl name against an algo def
class MAPTK_CORE_EXPORT invalid_name_exception
  : public algorithm_exception
{
  public:
    invalid_name_exception(std::string type,
                           std::string impl,
                           std::string reason) MAPTK_NOTHROW;
    virtual ~invalid_name_exception() MAPTK_NOTHROW;
};

} // end namespace maptk

#endif // MAPTK_CORE_EXCEPTIONS_ALGORITHM_H
