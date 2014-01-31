/*ckwg +5
 * Copyright 2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
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
 * \param name    A \c std::string name for this nested algorithm.
 * \param config  The \c maptk::config_block to add the given algorithm's
 *                options to.
 * \paraom algo   The cv pointer to the algorithm to pull configuration options from.
 */
template <typename T>
void get_nested_ocv_algo_configuration(std::string const& name,
                                                    config_block_sptr config,
                                                    cv::Ptr<T> algo);


} // end namespace ocv

} // end namespace maptk

#endif // MAPTK_OCV_OCV_ALGO_TOOLS_H
