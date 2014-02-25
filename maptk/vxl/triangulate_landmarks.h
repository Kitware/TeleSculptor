/*ckwg +5
 * Copyright 2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

/**
 * \file
 * \brief Header for VXL triangulate_landmarks algorithm
 */

#ifndef MAPTK_VXL_TRIANGULATE_LANDMARKS_H_
#define MAPTK_VXL_TRIANGULATE_LANDMARKS_H_

#include <maptk/vxl/vxl_config.h>

#include <maptk/core/algo/triangulate_landmarks.h>
#include <boost/scoped_ptr.hpp>

namespace maptk
{

namespace vxl
{

/// A class for triangulating landmarks from tracks and cameras using VXL
class MAPTK_VXL_EXPORT triangulate_landmarks
: public algo::algorithm_impl<triangulate_landmarks,
                              algo::triangulate_landmarks>
{
public:
  /// Constructor
  triangulate_landmarks();

  /// Destructor
  virtual ~triangulate_landmarks();

  /// Copy Constructor
  triangulate_landmarks(const triangulate_landmarks& other);

  /// Return the name of this implementation
  std::string impl_name() const { return "vxl"; }

  /// Get this algorithm's \link maptk::config_block configuration block \endlink
  virtual config_block_sptr get_configuration() const;
  /// Set this algorithm's properties via a config block
  virtual void set_configuration(config_block_sptr config);
  /// Check that the algorithm's currently configuration is valid
  virtual bool check_configuration(config_block_sptr config) const;

  /// Triangulate the landmark locations given sets of cameras and tracks
  /**
   * \param [in] cameras the cameras viewing the landmarks
   * \param [in] tracks the tracks to use as constraints
   * \param [in,out] landmarks the landmarks to triangulate
   *
   * This function only triangulates the landmarks with indicies in the
   * landmark map and which have support in the tracks and cameras
   */
  virtual void
  triangulate(camera_map_sptr cameras,
              track_set_sptr tracks,
              landmark_map_sptr& landmarks) const;

private:
  /// private implementation class
  class priv;
  boost::scoped_ptr<priv> d_;
};


} // end namespace vxl

} // end namespace maptk


#endif // MAPTK_VXL_TRIANGULATE_LANDMARKS_H_
