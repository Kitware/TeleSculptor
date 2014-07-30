/*ckwg +29
 * Copyright 2014 by Kitware, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither name of Kitware, Inc. nor the names of any contributors may be used
 *    to endorse or promote products derived from this software without specific
 *    prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS ``AS IS''
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHORS OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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

/// \brief Abstract base class for loop closure algorithms.
/**
 * Different algorithms can perform loop closure in a variety of ways, either
 * in attempt to make either short or long term closures. Similarly to
 * track_features, this class is designed to be called in an online fashion.
 */
class MAPTK_CORE_EXPORT close_loops
  : public algorithm_def<close_loops>
{
public:

  /// Return the name of this algorithm.
  virtual std::string type_name() const { return "close_loops"; }

  /// Attempt to perform closure operation and stitch tracks together.
  /**
   * \param [in] frame_number the frame number of the current frame
   * \param [in] image image data for the current frame
   * \param [in] input the input track set to stitch
   * \returns an updated set a tracks after the stitching operation
   */
  virtual track_set_sptr
  stitch( frame_id_t frame_number,
          image_container_sptr image,
          track_set_sptr input ) const = 0;

};

typedef boost::shared_ptr<close_loops> close_loops_sptr;

} // end namespace algo

} // end namespace maptk


#endif // MAPTK_ALGO_CLOSE_LOOPS_H_
