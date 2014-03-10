/*ckwg +5
 * Copyright 2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

/**
 * \file
 * \brief Instantiation of \link maptk::algo::algorithm_def algorithm_def<T>
 *        \endlink for \link maptk::algo::optimize_cameras
 *        optimize_cameras \endlink
 */

#include <maptk/core/algo/algorithm.txx>
#include <maptk/core/algo/optimize_cameras.h>


/// \cond DoxygenSuppress
INSTANTIATE_ALGORITHM_DEF(maptk::algo::optimize_cameras);
/// \endcond
