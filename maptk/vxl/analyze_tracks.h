/*ckwg +5
 * Copyright 2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#ifndef MAPTK_VXL_ANALYZE_TRACKS_H_
#define MAPTK_VXL_ANALYZE_TRACKS_H_

#include <maptk/vxl/vxl_config.h>

#include <maptk/core/algo/analyze_tracks.h>
#include <boost/scoped_ptr.hpp>

/**
 * \file
 * \brief Header for VXL analyze_tracks algorithm
 */

namespace maptk
{

namespace vxl
{

/// A class for triangulating landmarks from tracks and cameras using VXL
class MAPTK_VXL_EXPORT analyze_tracks
: public algo::algorithm_impl<analyze_tracks, algo::analyze_tracks>
{
public:

  /// Constructor
  analyze_tracks();

  /// Copy Constructor
  analyze_tracks(const analyze_tracks& other);

  /// Destructor
  virtual ~analyze_tracks();

  /// Return the name of this implementation
  std::string impl_name() const { return "vxl"; }

  /// Get this algorithm's \link maptk::config_block configuration block \endlink
  virtual config_block_sptr get_configuration() const;
  /// Set this algorithm's properties via a config block
  virtual void set_configuration(config_block_sptr config);
  /// Check that the algorithm's currently configuration is valid
  virtual bool check_configuration(config_block_sptr config) const;

  /// Output various information about the tracks stored in the input set
  /**
   * \param [in] track_set the tracks to analyze
   */
  virtual void
  analyze(track_set_sptr track_set) const;

  /// Output various information about the tracks stored in the input set
  /**
   *  This call can additionally use input imagery.
   *
   * \param [in] track_set the tracks to analyze
   * \param [in] image_data a list of images the tracks were computed on
   */
  virtual void
  analyze(track_set_sptr track_set,
          image_container_sptr_list image_data) const;

private:

  /// private implementation class
  class priv;
  boost::scoped_ptr<priv> d_;
};


} // end namespace vxl

} // end namespace maptk


#endif // MAPTK_VXL_TRIANGULATE_LANDMARKS_H_
