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

#include <maptk/core/config_block.h>
#include <maptk/core/exceptions/config_block.h>
#include <maptk/core/types.h>

#include <maptk/ocv/ocv_config.h>

#include <opencv2/core/core.hpp>

namespace maptk
{

namespace ocv
{

/// Add nested OpenCV algorithm's configuration options to the given \c config
/**
 * This includes an algorithm "type" parameter that defines what specific
 * algorithm we are nesting. If the given \c algo is not defined, we only set
 * a blank "type" option. If the given algorithm is defined, we set the "type"
 * parameter to the type of the algorithm given, and fill in that algorithm's
 * options under a subblock of its name.
 *
 * The \c config_block that we are expecting is at the block level for the
 * algorithm that this nested algorithm belongs to.
 *
 * \param name    A \c std::string name for this nested algorithm.
 * \param config  The \c maptk::config_block to add the given algorithm's
 *                options to.
 * \paraom algo   The cv pointer to the nested algorithm.
 */
MAPTK_OCV_EXPORT
void get_nested_ocv_algo_configuration(std::string const& name,
                                       config_block_sptr config,
                                       cv::Ptr<cv::Algorithm> algo);


/// Hidden helper method for setting nested OpenCV Algorithm parameters from a
/// \c config_block.
void set_nested_ocv_algo_configuration_helper(std::string const& name,
                                              config_block_sptr config,
                                              cv::Ptr<cv::Algorithm> &algo);


/// Set nested OpenCV algoruthm's parameters based on a given \c config
/**
 * We assume that what is pointed to be \c algo is an initialized OpenCV
 * algorithm class.
 *
 * \param algo_t  Type of OpenCV algorithm we are dealing with.
 * \param name    A \c std::string name for this nested algorithm. This should
 *                match the name used when \c get_nested_ocv_algo_configuration
 *                was called for this nested algorithm.
 * \param config  The \c maptk::config_block to set OpenCV property values
 *                from.
 * \paraom algo   The cv pointer to the algorithm to set configuration
 *                options to.
 */
template <typename algo_t>
MAPTK_OCV_EXPORT
void set_nested_ocv_algo_configuration(std::string const& name,
                                       config_block_sptr config,
                                       cv::Ptr<algo_t> &algo)
{
  cv::Ptr<cv::Algorithm> converted_algo(algo);
  set_nested_ocv_algo_configuration_helper(name, config, converted_algo);
  algo = cv::Ptr<algo_t>(converted_algo);
}


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
MAPTK_OCV_EXPORT
bool check_nested_ocv_algo_configuration(std::string const& name,
                                         config_block_sptr config);


} // end namespace ocv

} // end namespace maptk

#endif // MAPTK_OCV_OCV_ALGO_TOOLS_H
