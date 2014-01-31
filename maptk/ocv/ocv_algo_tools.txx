/**ckwg +5
 * Copyright 2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#ifndef MAPTK_OCV_OCV_ALGO_TOOLS_TXX
#define MAPTK_OCV_OCV_ALGO_TOOLS_TXX

namespace maptk
{

namespace ocv
{

/// Add nested OpenCV algorithm's configuration options to the given \c config
template <typename T>
void
get_nested_ocv_algo_configuration(std::string const& name,
                                  config_block_sptr config,
                                  cv::Ptr<T> algo)
{
  return;
}

} // end namespace ocv

} // end namespace maptk

#endif // MAPTK_OCV_OCV_ALGO_TOOLS_TXX
