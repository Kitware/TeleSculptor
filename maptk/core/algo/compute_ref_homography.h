/*ckwg +5
 * Copyright 2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

/**
 * \file
 * \brief compute_ref_homography algorithm definition
 */

#ifndef MAPTK_ALGO_COMPUTE_REF_HOMOGRAPHY_H_
#define MAPTK_ALGO_COMPUTE_REF_HOMOGRAPHY_H_

#include <maptk/core/core_config.h>

#include <vector>

#include <boost/shared_ptr.hpp>

#include <maptk/core/algo/algorithm.h>
#include <maptk/core/track_set.h>
#include <maptk/core/image_container.h>
#include <maptk/core/homography.h>

namespace maptk
{

namespace algo
{


/// Abstract base class for mapping each image to some reference image.
/**
 * This class differs from estimate_homographies in that estimate_homographies
 * simply performs a homography regression from matching feature points. This
 * class is designed to generate different types of homographies from input
 * feature tracks, which can transform each image back to the same coordinate
 * space derived from some initial refrerence image.
 */
class MAPTK_CORE_EXPORT compute_ref_homography
  : public algorithm_def<compute_ref_homography>
{
public:

  /// Return the name of this algorithm
  std::string type_name() const { return "compute_ref_homography"; }

  /// Estimate the transformation which maps some frame to a reference frame
  /**
   * Similarly to track_features, this class was designed to be called in
   * an online fashion for each sequential frame.
   *
   * \param [in]   frame_number frame identifier for the current frame
   * \param [in]   tracks the set of all tracked features from the image
   * \param return estimated homography
   */
  virtual f2f_homography_sptr
  estimate( frame_id_t frame_number,
            track_set_sptr tracks ) const = 0;

};


/// Shared pointer type of base compute_ref_homography algorithm definition class
typedef boost::shared_ptr<compute_ref_homography> compute_ref_homography_sptr;


} // end namespace algo

} // end namespace maptk


#endif // MAPTK_COMPUTE_REF_HOMOGRAPHY_H_
