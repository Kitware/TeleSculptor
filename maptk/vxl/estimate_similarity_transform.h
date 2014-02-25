/*ckwg +5
 * Copyright 2013-2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

/**
 * \file
 * \brief VXL version of similarity transform estimation
 */

#ifndef MAPTK_VXL_ESTIMATE_SIMILARITY_TRANSFORM_H
#define MAPTK_VXL_ESTIMATE_SIMILARITY_TRANSFORM_H

#include <maptk/vxl/vxl_config.h>

#include <maptk/core/algo/estimate_similarity_transform.h>
#include <maptk/core/vector.h>


namespace maptk
{

namespace vxl
{


/// VXL implementation of similarity transform estimation
class MAPTK_VXL_EXPORT estimate_similarity_transform
  : public algo::algorithm_impl<estimate_similarity_transform,
                                algo::estimate_similarity_transform>
{
public:
  /// Name of this implementation
  std::string impl_name() const { return "vxl"; }

  // No custom configuration at this time
  /// \cond Doxygen Suppress
  virtual void set_configuration(config_block_sptr /*config*/) { };
  virtual bool check_configuration(config_block_sptr /*config*/) const { return true; }
  /// \endcond

  /// Estimate the similarity transformation between two corresponding sets of
  /// 3d points
  /**
   * Complexity: Θ(n), where n = from.size() (from and to must be of equal
   * length).
   *
   * \param from List of length N of 3D points in the from space.
   * \param to   List of length N of 3D points in the to space.
   * \throws algorithm_exception When the from and to points sets are
   *                             misaligned, insufficient or degenerate.
   * \returns An estimated similarity transform mapping 3D points in the
   *          \c from space to points in the \c to space.
   */
  virtual similarity_d
  estimate_transform(std::vector<vector_3d> const& from,
                     std::vector<vector_3d> const& to) const;

};


} // end namespace vxl

} // end namespace maptk


#endif // MAPTK_VXL_ESTIMATE_SIMILARITY_TRANSFORM_H
