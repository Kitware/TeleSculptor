/*ckwg +5
 * Copyright 2011-2013 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#ifndef MAPTK_CORE_EXCEPTIONS_BASE_H
#define MAPTK_CORE_EXCEPTIONS_BASE_H

#include <maptk/core/core-config.h>
#include <string>

namespace maptk
{

/**
 * \brief The base class for all exceptions thrown from \ref config.
 * \ingroup exceptions
 */
class MAPTK_CORE_EXPORT maptk_core_base_exception
  : public std::exception
{
  public:
    /**
     * \brief Constructor.
     */
    maptk_core_base_exception() MAPTK_NOTHROW;
    /**
     * \brief Destructor.
     */
    virtual ~maptk_core_base_exception() MAPTK_NOTHROW;

    /**
     * \brief Description of the exception
     * \returns A string describing what went wrong.
     */
    char const* what() const MAPTK_NOTHROW;
  protected:
    std::string m_what;
};

}

#endif // MAPTK_CORE_EXCEPTIONS_BASE_H
