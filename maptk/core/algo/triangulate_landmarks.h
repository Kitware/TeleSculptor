/*ckwg +5
 * Copyright 2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#ifndef MAPTK_ALGO_TRIANGULATE_LANDMARKS_H_
#define MAPTK_ALGO_TRIANGULATE_LANDMARKS_H_

#include <maptk/core/algo/algorithm.h>
#include <maptk/core/track_set.h>
#include <maptk/core/camera_map.h>
#include <maptk/core/landmark_map.h>


/**
 * \file
 * \brief Header defining abstract \link maptk::algo::triangulate_landmarks
 *        triangulate landmarks \endlink algorithm
 */


namespace maptk
{

namespace algo
{

/// An abstract base class for triangulating landmarks
class MAPTK_CORE_EXPORT triangulate_landmarks
: public algorithm_def<triangulate_landmarks>
{
public:
  /// Return the name of this algorithm
  std::string type_name() const { return "triangulate_landmarks"; }

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
              landmark_map_sptr& landmarks) const = 0;
};


/// type definition for shared pointer to a triangulate landmarks algorithm
typedef boost::shared_ptr<triangulate_landmarks> triangulate_landmarks_sptr;


} // end namespace algo

} // end namespace maptk


#endif // MAPTK_ALGO_TRIANGULATE_LANDMARKS_H_
