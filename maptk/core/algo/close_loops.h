/*ckwg +5
 * Copyright 2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#ifndef MAPTK_ALGO_CLOSE_LOOPS_H_
#define MAPTK_ALGO_CLOSE_LOOPS_H_

#include <maptk/core/core_config.h>

#include <boost/shared_ptr.hpp>

#include <maptk/core/algo/algorithm.h>
#include <maptk/core/image_container.h>
#include <maptk/core/track_set.h>

#include <ostream>

/**
 * \file
 * \brief Header defining abstract \link maptk::algo::close_loops track
 *        analyzer \endlink algorithm
 */

namespace maptk
{

namespace algo
{

/// \brief An abstract base class for algorithms which attempt to find
/// more feature track matches over a longer period of time compared with
/// looking only at the previous frame.
///
/// Similarly to track_features, this class is designed to be called
/// in an online fashion.
class MAPTK_CORE_EXPORT close_loops
  : public algorithm_def<close_loops>
{
public:

  /// Return the name of this algorithm.
  std::string type_name() const { return "close_loops"; }

  /// Attempt to find more track matches to aid in loop closure
  /// and stitch them together in the output track set.
  /**
   * \param [in] frame_number the frame number of the current frame
   * \param [in] input the input track set to stitch
   * \returns an updated set a tracks after the stitching operation
   */
  virtual track_set_sptr
  stitch( frame_id_t frame_number,
          track_set_sptr input ) const = 0;

};

typedef boost::shared_ptr<close_loops> close_loops_sptr;

} // end namespace algo

} // end namespace maptk


#endif // MAPTK_ALGO_CLOSE_LOOPS_H_
