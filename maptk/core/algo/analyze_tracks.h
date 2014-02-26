/*ckwg +5
 * Copyright 2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#ifndef MAPTK_ALGO_ANALYZE_TRACKS_H_
#define MAPTK_ALGO_ANALYZE_TRACKS_H_

#include <maptk/core/core_config.h>

#include <boost/shared_ptr.hpp>

#include <maptk/core/algo/algorithm.h>
#include <maptk/core/image_container.h>
#include <maptk/core/track_set.h>

#include <ostream>

/**
 * \file
 * \brief Header defining abstract \link maptk::algo::analyze_tracks track
 *        analyzer \endlink algorithm
 */

namespace maptk
{

namespace algo
{

/// An abstract base class for algorithms which output various human readable
/// statistics about track sets, to aid with both debugging and algorithm
/// tuning (either manual or automatic).
class MAPTK_CORE_EXPORT analyze_tracks
  : public algorithm_def<analyze_tracks>
{
public:

  typedef std::ostream stream_t;

  /// Return the name of this algorithm.
  std::string type_name() const { return "analyze_tracks"; }

  /// Output various information about the tracks stored in the input set.
  /**
   * \param [in] track_set the tracks to analyze
   * \param [in] stream an output stream to write data onto
   */
  virtual void
  print_info(track_set_sptr track_set,
             stream_t& stream = std::cout) const = 0;

  /// Write features tracks on top of the input images.
  /**
   * \param [in] track_set the tracks to analyze
   * \param [in] image_data a list of images the tracks were computed on
   */
  virtual void
  write_images(track_set_sptr track_set,
               image_container_sptr_list image_data) const = 0;

};

typedef boost::shared_ptr<analyze_tracks> analyze_tracks_sptr;

} // end namespace algo

} // end namespace maptk


#endif // MAPTK_ALGO_TRACK_FEATURES_H_
