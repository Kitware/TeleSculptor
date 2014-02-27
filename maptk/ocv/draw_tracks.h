/*ckwg +5
 * Copyright 2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#ifndef MAPTK_OCV_DRAW_TRACKS_H_
#define MAPTK_OCV_DRAW_TRACKS_H_

#include <maptk/ocv/ocv_config.h>

#include <maptk/core/algo/draw_tracks.h>
#include <boost/scoped_ptr.hpp>

/**
 * \file
 * \brief Header for OCV draw_tracks algorithm
 */

namespace maptk
{

namespace ocv
{

/// A class for outputting various debug info about feature tracks
class MAPTK_OCV_EXPORT draw_tracks
: public algo::algorithm_impl<draw_tracks, algo::draw_tracks>
{
public:

  /// Constructor
  draw_tracks();

  /// Copy Constructor
  draw_tracks(const draw_tracks& other);

  /// Destructor
  virtual ~draw_tracks();

  /// Return the name of this implementation
  std::string impl_name() const { return "ocv"; }

  /// Get this algorithm's \link maptk::config_block configuration block \endlink
  virtual config_block_sptr get_configuration() const;
  /// Set this algorithm's properties via a config block
  virtual void set_configuration(config_block_sptr config);
  /// Check that the algorithm's currently configuration is valid
  virtual bool check_configuration(config_block_sptr config) const;

  /// Write features tracks on top of the input images.
  /**
   * \param [in] track_set the tracks to analyze
   * \param [in] image_data a list of images the tracks were computed on
   */
  virtual void
  draw(track_set_sptr track_set,
       image_container_sptr_list image_data) const;

private:

  /// private implementation class
  class priv;
  boost::scoped_ptr<priv> d_;
};


} // end namespace ocv

} // end namespace maptk


#endif // MAPTK_OCV_DRAW_TRACKS_H_
