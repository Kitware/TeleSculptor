/*ckwg +5
 * Copyright 2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

/**
 * \file
 * \brief Header defining homography overlap helper functions
 */

#ifndef MAPTK_VXL_COMPUTE_HOMOGRAPHY_OVERLAP_H_
#define MAPTK_VXL_COMPUTE_HOMOGRAPHY_OVERLAP_H_


#include <vnl/vnl_double_3x3.h>


namespace maptk
{

namespace vxl
{


/// Return the overlap between two images.
/**
 * This function assumes that a homography perfectly describes the
 * transformation between these 2 images (in some reference coordinate
 * system). The overlap is returned as a percentage.
 */
double
overlap( const vnl_double_3x3& h, const unsigned ni, const unsigned nj );


} // end namespace vxl

} // end namespace maptk


#endif // MAPTK_VXL_COMPUTE_HOMOGRAPHY_OVERLAP_H_
