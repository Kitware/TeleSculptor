/*ckwg +5
 * Copyright 2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

/**
 * \file
 * \brief Helper utility functions for getting, setting and checking properties
 *        between OpenCV algorithm properties and maptk::config_block objects.
 */

#ifndef MAPTK_OCV_OCV_ALGO_TOOLS_H
#define MAPTK_OCV_OCV_ALGO_TOOLS_H

#include <string>

#include <maptk/core/types.h>

namespace maptk
{

namespace ocv
{

/// Add nested OpenCV algorithm's configuration options to the given \c config
/**
 * We assume that what is pointed to by \c algo is an initialized OpenCV
 * algorithm class.
 *
 * \param T       Type of OpenCV algorithm we are dealing with.
 *
 * \param name    A \c std::string name for this nested algorithm.
 * \param config  The \c maptk::config_block to add the given algorithm's
 *                options to.
 * \paraom algo   The cv pointer to the algorithm to pull configuration
 *                options from.
 */
template <typename T>
void get_nested_ocv_algo_configuration(std::string const& name,
                                       config_block_sptr config,
                                       cv::Ptr<T> algo);

/// Set nested OpenCV algoruthm's parameters based on a given \c config
/**
 * We assume that what is pointed to be \c algo is an initialized OpenCV
 * algorithm class.
 *
 * \param T       Type of OpenCV algorithm we are dealing with.
 * \param name    A \c std::string name for this nested algorithm. This should
 *                match the name used when \c get_nested_ocv_algo_configuration
 *                was called for this nested algorithm.
 * \param config  The \c maptk::config_block to set OpenCV property values
 *                from.
 * \paraom algo   The cv pointer to the algorithm to set configuration
 *                options to.
 */
template <typename T>
void set_nested_ocv_algo_configuration(std::string const& name,
                                       config_block_sptr config,
                                       cv::Ptr<T> algo);

/// Basic check of nested OpenCV algorithm configuration in the given \c config
/**
 * Of all nested algorithm configuration properties that can be encoded within
 * a \c config, check that they are present and that the value in the \c config
 * is castable to the nested algorithm parameter's expected type.
 *
 * We assume that what is pointed to be \c algo is an initialized OpenCV
 * algorithm class.
 *
 * \param T       Type of OpenCV algorithm we are dealing with.
 * \param name    A \c std::string name for this nested algorithm. This should
 *                match the name used whe \c get_nested_ocv_algo_configuration
 *                was called for this nested algorithm.
 * \param config  The \c maptk::config_block to check.
 * \param algo    The cv pointer to the algorithn to use as a reference to
 *                check the \c config.
 */
template <typename T>
bool check_nested_ocv_algo_configuration(std::string const& name,
                                         config_block_sptr config,
                                         cv::Ptr<T> algo);


} // end namespace ocv

} // end namespace maptk

#endif // MAPTK_OCV_OCV_ALGO_TOOLS_H
