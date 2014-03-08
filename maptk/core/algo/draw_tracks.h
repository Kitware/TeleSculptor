/*ckwg +5
 * Copyright 2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#ifndef MAPTK_ALGO_DRAW_TRACKS_H_
#define MAPTK_ALGO_DRAW_TRACKS_H_

#include <maptk/core/core_config.h>

#include <boost/shared_ptr.hpp>

#include <maptk/core/algo/algorithm.h>
#include <maptk/core/image_container.h>
#include <maptk/core/track_set.h>

#include <ostream>

/**
 * \file
 * \brief Header defining an abstract \link maptk::algo::draw_tracks track
 *        drawing \endlink algorithm
 */

namespace maptk
{

namespace algo
{


/// An abstract base class for algorithms which draw tracks on top of
/// images in various ways, for analyzing results.
class MAPTK_CORE_EXPORT draw_tracks
  : public algorithm_def<draw_tracks>
{
public:

  /// Return the name of this algorithm.
  std::string type_name() const { return "draw_tracks"; }

  /// Draw features tracks on top of the input images.
  /**
   * This process can either be called in an offline fashion, where all
   * tracks and images are provided to the function on the first call,
   * or in an online fashion where only new images should be provided
   * on sequential calls.
   *
   * \param [in] track_set the tracks to draw
   * \param [in] image_data a list of images the tracks were computed over
   * \param returns a pointer to the last image generated
   */
  virtual image_container_sptr
  draw(track_set_sptr track_set,
       image_container_sptr_list image_data) = 0;

  /// Draw features tracks on top of the input images.
  /**
   * This function additionally consumes a second track set for which can
   * optionally be used to display additional information to provide a
   * comparison between the two track sets.
   *
   * \param [in] track_set the main tracks to draw
   * \param [in] comparison_set the comparison track set
   * \param [in] image_data a list of images the tracks were computed over
   * \param returns a pointer to the last image generated
   */
  virtual image_container_sptr
  draw(track_set_sptr track_set,
       track_set_sptr comparison_set,
       image_container_sptr_list image_data) = 0;

};


/// A smart pointer to a draw_tracks instance.
typedef boost::shared_ptr<draw_tracks> draw_tracks_sptr;


} // end namespace algo

} // end namespace maptk


#endif // MAPTK_ALGO_DRAW_TRACKS_H_
